#!/usr/bin/env python3
"""Assert the DroneCAN round trip works: /wheel_cmd out, /wheel_feedback back.

Drives rp1_dronecan_bridge with a known WheelCommand and checks the WheelFeedback that comes
back off the CAN bus, so it only passes if the whole path is intact: bridge encoding
esc.RawCommand, real DroneCAN framing on the wire, sim_vesc_node decoding it, its esc.Status
replies, and the bridge turning those back into ROS messages. Run it against
`sim_vesc_node.py --iface vcan0` (see README.md here); `robotpicks.sh smoke dronecan` wires up
both sides and calls this.

Two phases, because "telemetry appears" is a weaker claim than "the robot stops when told
nothing":

  1. drive  -- publish `command` and expect every wheel to report command * max_rpm
  2. coast  -- stop publishing entirely and expect every wheel back to 0, which exercises the
               bridge's command_timeout failsafe rather than a commanded zero

Deliberately an rclpy subscriber rather than `ros2 topic echo`: the CLI has shown flaky
hangs/false failures in this sandbox when piped or wrapped in timeout (see rp1/CLAUDE.md).

Exits 0 if both phases pass, 1 with a diagnosis otherwise.
"""

import argparse
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from rp1_msgs.msg import WheelCommand, WheelFeedback

NUM_WHEELS = 4


class LoopbackChecker(Node):

    def __init__(self, command: float, publish_rate_hz: float):
        super().__init__('dronecan_loopback_check')
        self._command = command
        self._driving = True
        self._rpm = {}                  # wheel_index -> latest rpm reported
        self._pub = self.create_publisher(WheelCommand, 'wheel_cmd', 10)
        # Queue deep enough not to drop samples from 4 wheels at the sim's status rate.
        self.create_subscription(WheelFeedback, 'wheel_feedback', self._on_feedback, 50)
        self.create_timer(1.0 / publish_rate_hz, self._on_tick)

    def _on_tick(self) -> None:
        # In the coast phase publish nothing at all: a commanded zero would prove only that
        # zero propagates, not that the bridge fails safe when commands stop arriving.
        if not self._driving:
            return
        msg = WheelCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.velocity = [self._command] * NUM_WHEELS
        self._pub.publish(msg)

    def _on_feedback(self, msg: WheelFeedback) -> None:
        self._rpm[msg.wheel_index] = msg.rpm

    def coast(self) -> None:
        """Stop commanding and forget past samples, so the next check can't pass on stale data."""
        self._driving = False
        self._rpm.clear()

    def settled_at(self, target_rpm: float, tolerance_rpm: float) -> bool:
        if len(self._rpm) < NUM_WHEELS:
            return False
        return all(abs(rpm - target_rpm) <= tolerance_rpm for rpm in self._rpm.values())

    def report(self) -> str:
        if not self._rpm:
            return 'no /wheel_feedback received at all'
        seen = ', '.join(f'wheel {i}: {self._rpm[i]:.1f} rpm' for i in sorted(self._rpm))
        missing = sorted(set(range(NUM_WHEELS)) - set(self._rpm))
        if missing:
            return f'{seen} (no report from wheel(s) {missing})'
        return seen


def wait_until(node, predicate, timeout_sec: float) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return True
    return False


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--command', type=float, default=0.5,
                        help='normalized WheelCommand to drive with (default: 0.5)')
    parser.add_argument('--max-rpm', type=float, default=3000.0,
                        help="sim_vesc_node's MAX_RPM, which sets the expected rpm (default: 3000)")
    parser.add_argument('--tolerance-rpm', type=float, default=50.0,
                        help='allowed deviation from the expected rpm (default: 50)')
    parser.add_argument('--timeout', type=float, default=20.0,
                        help='seconds to allow each phase to settle (default: 20)')
    parser.add_argument('--publish-rate-hz', type=float, default=20.0)
    args = parser.parse_args()

    expected = args.command * args.max_rpm

    rclpy.init()
    node = LoopbackChecker(args.command, args.publish_rate_hz)
    rc = 0
    try:
        print(f'[check] phase 1/2: driving {args.command}, expecting '
              f'{expected:.0f} rpm on all {NUM_WHEELS} wheels', flush=True)
        if not wait_until(node, lambda: node.settled_at(expected, args.tolerance_rpm),
                          args.timeout):
            print(f'[check] FAIL: wheels never reached {expected:.0f} rpm '
                  f'(+/-{args.tolerance_rpm:.0f}) within {args.timeout:.0f}s -- {node.report()}',
                  file=sys.stderr)
            return 1
        print(f'[check] phase 1 ok -- {node.report()}', flush=True)

        print('[check] phase 2/2: commands stopped, expecting the timeout failsafe to zero them',
              flush=True)
        node.coast()
        if not wait_until(node, lambda: node.settled_at(0.0, args.tolerance_rpm), args.timeout):
            print(f'[check] FAIL: wheels did not return to 0 rpm within {args.timeout:.0f}s '
                  f'after commands stopped -- {node.report()}', file=sys.stderr)
            return 1
        print(f'[check] phase 2 ok -- {node.report()}', flush=True)
        print('[check] PASS: DroneCAN loopback and command-timeout failsafe both good')
    except (KeyboardInterrupt, ExternalShutdownException):
        rc = 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
