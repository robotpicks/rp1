"""Xbox Series X controller -> /cmd_vel, gated by a deadman button.

Publishes geometry_msgs/Twist only while the configured deadman button is held; otherwise (or
if /joy goes stale) it publishes zero velocity. See config/joy_xbox_series_x.yaml for the axis
and button mapping, which should be verified against the real controller before trusting it.
"""

import rclpy
from geometry_msgs.msg import Twist
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
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
