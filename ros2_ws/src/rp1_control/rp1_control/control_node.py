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
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
