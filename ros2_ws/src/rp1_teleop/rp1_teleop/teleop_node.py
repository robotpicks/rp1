"""Xbox Series X controller -> /cmd_vel, gated by a deadman button.

Publishes geometry_msgs/Twist only while the configured deadman button is held; otherwise (or
if /joy goes stale) it publishes zero velocity. See config/joy_xbox_series_x.yaml for the axis
and button mapping, which should be verified against the real controller before trusting it.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy


class TeleopNode(Node):

    def __init__(self):
        super().__init__('teleop_node')

        self.declare_parameter('axis_linear', 1)
        self.declare_parameter('axis_angular', 3)
        self.declare_parameter('scale_linear', 1.0)
        self.declare_parameter('scale_angular', 2.0)
        self.declare_parameter('invert_linear', False)
        self.declare_parameter('invert_angular', False)
        self.declare_parameter('deadman_button', 4)
        self.declare_parameter('joy_timeout_sec', 0.5)

        self._axis_linear = self.get_parameter('axis_linear').value
        self._axis_angular = self.get_parameter('axis_angular').value
        self._scale_linear = self.get_parameter('scale_linear').value
        self._scale_angular = self.get_parameter('scale_angular').value
        self._sign_linear = -1.0 if self.get_parameter('invert_linear').value else 1.0
        self._sign_angular = -1.0 if self.get_parameter('invert_angular').value else 1.0
        self._deadman_button = self.get_parameter('deadman_button').value
        self._joy_timeout = self.get_parameter('joy_timeout_sec').value

        self._last_joy_time = None

        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Joy, 'joy', self._on_joy, 10)
        self.create_timer(0.1, self._watchdog)

        self.get_logger().info('rp1_teleop ready (deadman button %d must be held to drive)'
                                % self._deadman_button)

    def _on_joy(self, msg: Joy) -> None:
        self._last_joy_time = self.get_clock().now()

        deadman_held = (len(msg.buttons) > self._deadman_button
                         and msg.buttons[self._deadman_button] == 1)

        twist = Twist()
        if deadman_held:
            linear = msg.axes[self._axis_linear] if len(msg.axes) > self._axis_linear else 0.0
            angular = msg.axes[self._axis_angular] if len(msg.axes) > self._axis_angular else 0.0
            twist.linear.x = self._sign_linear * linear * self._scale_linear
            twist.angular.z = self._sign_angular * angular * self._scale_angular

        self._cmd_vel_pub.publish(twist)

    def _watchdog(self) -> None:
        if self._last_joy_time is None:
            return
        stale = (self.get_clock().now() - self._last_joy_time).nanoseconds / 1e9
        if stale > self._joy_timeout:
            self._cmd_vel_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
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
