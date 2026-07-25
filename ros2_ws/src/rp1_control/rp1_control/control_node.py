"""/cmd_vel -> per-wheel skid-steer WheelCommand for the rp1 MVP (4 diff wheels, no steering).

Left-side wheels (front-left, rear-left) share one speed, right-side wheels share another:
  v_left  = v - w * track_width / 2
  v_right = v + w * track_width / 2

Both are scaled by max_wheel_speed into the normalized [-1, 1] WheelCommand range; if either
side would exceed that range, both are scaled down together so the requested turn ratio is
preserved rather than clipped independently.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rp1_msgs.msg import WheelCommand


class ControlNode(Node):

    def __init__(self):
        super().__init__('control_node')

        self.declare_parameter('track_width', 0.5)
        self.declare_parameter('max_wheel_speed', 1.0)
        self.declare_parameter('cmd_vel_timeout_sec', 0.5)

        self._track_width = self.get_parameter('track_width').value
        self._max_wheel_speed = self.get_parameter('max_wheel_speed').value
        self._cmd_vel_timeout = self.get_parameter('cmd_vel_timeout_sec').value

        self._last_cmd_time = None

        self._wheel_cmd_pub = self.create_publisher(WheelCommand, 'wheel_cmd', 10)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)
        self.create_timer(0.1, self._watchdog)

        self.get_logger().info('rp1_control ready (track_width=%.3f m, max_wheel_speed=%.3f m/s)'
                                % (self._track_width, self._max_wheel_speed))

    def _publish(self, left: float, right: float) -> None:
        if self._max_wheel_speed > 0.0:
            norm_left = left / self._max_wheel_speed
            norm_right = right / self._max_wheel_speed
        else:
            norm_left = norm_right = 0.0

        peak = max(abs(norm_left), abs(norm_right), 1.0)
        norm_left /= peak
        norm_right /= peak

        msg = WheelCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.velocity[WheelCommand.WHEEL_FRONT_LEFT] = norm_left
        msg.velocity[WheelCommand.WHEEL_REAR_LEFT] = norm_left
        msg.velocity[WheelCommand.WHEEL_FRONT_RIGHT] = norm_right
        msg.velocity[WheelCommand.WHEEL_REAR_RIGHT] = norm_right
        self._wheel_cmd_pub.publish(msg)

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_cmd_time = self.get_clock().now()
        v = msg.linear.x
        w = msg.angular.z
        left = v - w * self._track_width / 2.0
        right = v + w * self._track_width / 2.0
        self._publish(left, right)

    def _watchdog(self) -> None:
        if self._last_cmd_time is None:
            return
        stale = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if stale > self._cmd_vel_timeout:
            self._publish(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
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
