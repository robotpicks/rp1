"""Adapter: rp1_msgs/WheelFeedback -> sensor_msgs/BatteryState for the generic elrs_driver.

The ExpressLRS driver (`elrs_driver`, from the elrs_ros submodule) is robot-agnostic: it sends
handset battery telemetry from a standard `sensor_msgs/BatteryState`. rp1's per-wheel telemetry
lives in `rp1_msgs/WheelFeedback` (voltage/current per VESC), so this thin node aggregates the 4
wheels into one pack-level BatteryState.

This is the *only* rp1-specific code in the ELRS path -- everything else (CRSF parsing, RC->Joy,
serial, failsafe, telemetry framing) is the generic elrs_ros package. See rp1/CLAUDE.md.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rp1_msgs.msg import WheelFeedback
from sensor_msgs.msg import BatteryState


class WheelFeedbackToBattery(Node):

    def __init__(self):
        super().__init__('wheel_feedback_to_battery')

        # 'mean' of the wheels that have reported, or a specific wheel index '0'..'3'.
        self.declare_parameter('battery_source', 'mean')
        self.declare_parameter('publish_rate_hz', 5.0)
        self._source = str(self.get_parameter('battery_source').value)

        self._voltage = [None, None, None, None]
        self._current = [None, None, None, None]

        self._pub = self.create_publisher(BatteryState, 'battery', 10)
        self.create_subscription(WheelFeedback, 'wheel_feedback', self._on_feedback, 10)

        rate = self.get_parameter('publish_rate_hz').value
        self.create_timer(1.0 / rate, self._tick)

    def _on_feedback(self, msg: WheelFeedback) -> None:
        if 0 <= msg.wheel_index < 4:
            self._voltage[msg.wheel_index] = float(msg.voltage)
            self._current[msg.wheel_index] = float(msg.current)

    def _aggregate(self, values):
        present = [v for v in values if v is not None]
        if not present:
            return None
        if self._source == 'mean':
            return sum(present) / len(present)
        try:
            idx = int(self._source)
        except ValueError:
            return sum(present) / len(present)
        if 0 <= idx < 4 and values[idx] is not None:
            return values[idx]
        return sum(present) / len(present)

    def _tick(self) -> None:
        voltage = self._aggregate(self._voltage)
        if voltage is None:
            return  # no WheelFeedback seen yet
        current = self._aggregate(self._current) or 0.0
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = float(voltage)
        # ROS convention is negative-when-discharging; VESC current sign varies (regen can flip
        # it), and elrs_driver reports the magnitude anyway, so pass the pack sum through as-is.
        msg.current = float(current)
        msg.percentage = float('nan')   # state-of-charge not estimated yet
        msg.present = True
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WheelFeedbackToBattery()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C, or a launch/systemd supervisor sending SIGTERM: rclpy's signal handler
        # shuts the context down and spin() reports it as ExternalShutdownException.
        # That is a normal stop, so swallow it and exit 0 -- otherwise every clean
        # shutdown looks like a crash to whatever is supervising the node.
        pass
    except RuntimeError:
        # The same stop, different symptom: if the signal lands while the executor is building
        # its wait set, rclpy invalidates the context underneath itself and raises RCLError
        # ("the given context is not valid") instead. It is a RuntimeError subclass that only
        # exists in a private module, so match the base and gate on the context actually being
        # gone -- a RuntimeError with a live context is a real fault and must propagate.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        # try_shutdown(), not shutdown(): the context is already down in the
        # ExternalShutdownException case and shutdown() would raise on top of it.
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
