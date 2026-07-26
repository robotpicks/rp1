#!/usr/bin/env python3
"""Assert the DroneCAN round trip works: /cmd_vel out, wheel state and ESC telemetry back.

Drives the real bringup pipeline with a known TwistStamped and checks what comes back off the
CAN bus, so it only passes if the whole path is intact: diff_drive_controller's skid-steer
kinematics, rp1_hardware_interface encoding esc.RPMCommand, real DroneCAN framing on the wire,
sim_vesc_node decoding it, its esc.Status replies, and the hardware component turning those back
into ros2_control state interfaces. Run it against `sim_vesc_node.py --iface vcan0` plus
`ros2 launch rp1_bringup rp1_mvp.launch.py can_iface:=vcan0 teleop:=false` (see README.md here);
`robotpicks.sh smoke dronecan` wires up both sides and calls this.

Three checks, because each is a claim the others don't make:

  1. drive     -- publish `speed` and expect every drive joint to report speed/wheel_radius rad/s
                  in /joint_states. This is the one that would catch a units error: the value
                  survives rad/s -> mechanical RPM -> ERPM -> the sim -> mechanical RPM -> rad/s
                  only if the gear ratio and pole-pair handling agree at both ends.
  2. telemetry -- expect a real per-ESC voltage on /dynamic_joint_states. Separate path from the
                  joint states above (<gpio> state interfaces, not joint interfaces) and the one
                  the ELRS handset telemetry hangs off, so a joints-only check would miss it
                  break. This phase is why the checker needs the real hardware component and not
                  use_mock:=true: mock_components leaves unclaimed gpio states at NaN, whereas
                  sim_vesc_node reports an actual pack voltage over the wire.
  3. coast     -- stop publishing entirely and expect every wheel back to 0, which exercises
                  diff_drive_controller's cmd_vel_timeout rather than a commanded zero.

Deliberately an rclpy subscriber rather than `ros2 topic echo`: the CLI has shown flaky
hangs/false failures in this sandbox when piped or wrapped in timeout (see rp1/CLAUDE.md).

Exits 0 if all three pass, 1 with a diagnosis otherwise.
"""

import argparse
import math
import sys
import time

import rclpy
from control_msgs.msg import DynamicJointState
from geometry_msgs.msg import TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

# Must match the joint names in rp1_hardware_interface/urdf/rp1_drive.urdf and the wheel lists in
# rp1_bringup/config/rp1_controllers.yaml.
DRIVE_JOINTS = (
    'drive_front_left', 'drive_front_right', 'drive_rear_left', 'drive_rear_right')
# The <gpio> names in the same URDF.
ESC_NAMES = ('esc_front_left', 'esc_front_right', 'esc_rear_left', 'esc_rear_right')

CMD_VEL_TOPIC = '/diff_drive_controller/cmd_vel'


class LoopbackChecker(Node):

    def __init__(self, speed: float, publish_rate_hz: float):
        super().__init__('dronecan_loopback_check')
        self._speed = speed
        self._driving = True
        self._velocity = {}             # joint name -> latest rad/s reported
        self._voltage = {}              # gpio name  -> latest volts reported
        self._pub = self.create_publisher(TwistStamped, CMD_VEL_TOPIC, 10)
        # Queues deep enough not to drop samples at the broadcaster's publish rate.
        self.create_subscription(JointState, '/joint_states', self._on_joint_states, 50)
        self.create_subscription(
            DynamicJointState, '/dynamic_joint_states', self._on_dynamic_states, 50)
        self.create_timer(1.0 / publish_rate_hz, self._on_tick)

    def _on_tick(self) -> None:
        # In the coast phase publish nothing at all: a commanded zero would prove only that zero
        # propagates, not that the controller fails safe when commands stop arriving.
        if not self._driving:
            return
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = self._speed
        self._pub.publish(msg)

    def _on_joint_states(self, msg: JointState) -> None:
        for name, velocity in zip(msg.name, msg.velocity):
            if name in DRIVE_JOINTS:
                self._velocity[name] = velocity

    def _on_dynamic_states(self, msg: DynamicJointState) -> None:
        for name, values in zip(msg.joint_names, msg.interface_values):
            if name not in ESC_NAMES:
                continue
            for interface, value in zip(values.interface_names, values.values):
                if interface == 'voltage':
                    self._voltage[name] = value

    def coast(self) -> None:
        """Stop commanding and forget past samples, so the next check can't pass on stale data."""
        self._driving = False
        self._velocity.clear()

    def settled_at(self, target: float, tolerance: float) -> bool:
        if len(self._velocity) < len(DRIVE_JOINTS):
            return False
        return all(abs(v - target) <= tolerance for v in self._velocity.values())

    def telemetry_seen(self) -> bool:
        # Presence alone is too weak: EscTelemetry.voltage starts at 0.0, so an esc.Status that
        # stopped decoding would still show up on the topic, at zero. Require a real reading.
        return (len(self._voltage) == len(ESC_NAMES)
                and all(math.isfinite(v) and v > 0.0 for v in self._voltage.values()))

    def report(self) -> str:
        if not self._velocity:
            return 'no /joint_states received for any drive joint'
        seen = ', '.join(
            f'{name}: {self._velocity[name]:.3f} rad/s' for name in sorted(self._velocity))
        missing = sorted(set(DRIVE_JOINTS) - set(self._velocity))
        if missing:
            return f'{seen} (nothing from {", ".join(missing)})'
        return seen

    def telemetry_report(self) -> str:
        if not self._voltage:
            return 'no per-ESC voltage on /dynamic_joint_states at all'
        seen = ', '.join(f'{name}: {self._voltage[name]:.1f} V' for name in sorted(self._voltage))
        missing = sorted(set(ESC_NAMES) - set(self._voltage))
        if missing:
            return f'{seen} (nothing from {", ".join(missing)})'
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
    parser.add_argument('--speed', type=float, default=0.5,
                        help='forward speed to command, m/s (default: 0.5)')
    parser.add_argument('--wheel-radius', type=float, default=0.1,
                        help="diff_drive_controller's wheel_radius, which turns the commanded "
                             'speed into the expected wheel rad/s (default: 0.1)')
    parser.add_argument('--tolerance', type=float, default=0.25,
                        help='allowed deviation from the expected rad/s (default: 0.25)')
    parser.add_argument('--timeout', type=float, default=20.0,
                        help='seconds to allow each phase to settle (default: 20)')
    parser.add_argument('--publish-rate-hz', type=float, default=20.0)
    args = parser.parse_args()

    expected = args.speed / args.wheel_radius

    rclpy.init()
    node = LoopbackChecker(args.speed, args.publish_rate_hz)
    rc = 0
    try:
        print(f'[check] phase 1/3: driving {args.speed} m/s, expecting {expected:.2f} rad/s '
              f'on all {len(DRIVE_JOINTS)} drive joints', flush=True)
        if not wait_until(node, lambda: node.settled_at(expected, args.tolerance), args.timeout):
            print(f'[check] FAIL: wheels never reached {expected:.2f} rad/s '
                  f'(+/-{args.tolerance}) within {args.timeout:.0f}s -- {node.report()}',
                  file=sys.stderr)
            return 1
        print(f'[check] phase 1 ok -- {node.report()}', flush=True)

        print('[check] phase 2/3: expecting per-ESC voltage telemetry', flush=True)
        if not wait_until(node, node.telemetry_seen, args.timeout):
            print(f'[check] FAIL: per-ESC telemetry incomplete within {args.timeout:.0f}s -- '
                  f'{node.telemetry_report()}', file=sys.stderr)
            return 1
        print(f'[check] phase 2 ok -- {node.telemetry_report()}', flush=True)

        print('[check] phase 3/3: commands stopped, expecting cmd_vel_timeout to zero them',
              flush=True)
        node.coast()
        if not wait_until(node, lambda: node.settled_at(0.0, args.tolerance), args.timeout):
            print(f'[check] FAIL: wheels did not return to 0 rad/s within {args.timeout:.0f}s '
                  f'after commands stopped -- {node.report()}', file=sys.stderr)
            return 1
        print(f'[check] phase 3 ok -- {node.report()}', flush=True)
        print('[check] PASS: DroneCAN loopback, ESC telemetry and cmd_vel timeout all good')
    except (KeyboardInterrupt, ExternalShutdownException):
        rc = 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
