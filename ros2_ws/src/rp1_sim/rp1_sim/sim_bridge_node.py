"""Pure-ROS2 stand-in for rp1_dronecan_bridge: no CAN, no DroneCAN, no VESCs.

Subscribes `rp1_msgs/WheelCommand` (the same topic rp1_dronecan_bridge would consume),
integrates simple skid-steer dynamics, and publishes `rp1_msgs/WheelFeedback` plus
`nav_msgs/Odometry` + an `odom` -> `base_link` TF. This lets the whole rp1_teleop -> rp1_control
graph be driven and watched (`ros2 topic echo`, rviz2) with zero hardware -- swap this node in
for rp1_dronecan_bridge (see rp1_bringup/launch/rp1_mvp_sim.launch.py) to check the ROS2
architecture end to end before any CAN bus exists.

The wheel-speed -> rpm/voltage/current numbers are a plausible stand-in for telemetry shape,
not a physically accurate motor model.
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rp1_msgs.msg import WheelCommand, WheelFeedback
from tf2_ros import TransformBroadcaster

NOMINAL_VOLTAGE = 24.0
CURRENT_PER_UNIT_VELOCITY = 10.0


class SimBridgeNode(Node):

    def __init__(self):
        super().__init__('sim_bridge_node')

        self.declare_parameter('track_width', 0.5)
        self.declare_parameter('max_wheel_speed', 1.0)
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('update_rate_hz', 50.0)
        self.declare_parameter('cmd_timeout_sec', 0.2)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')

        self._track_width = self.get_parameter('track_width').value
        self._max_wheel_speed = self.get_parameter('max_wheel_speed').value
        self._wheel_radius = self.get_parameter('wheel_radius').value
        self._cmd_timeout = self.get_parameter('cmd_timeout_sec').value
        self._publish_tf = self.get_parameter('publish_tf').value
        self._odom_frame_id = self.get_parameter('odom_frame_id').value
        self._base_frame_id = self.get_parameter('base_frame_id').value

        self._latest_velocity = [0.0, 0.0, 0.0, 0.0]
        self._last_cmd_time = None
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        self._feedback_pub = self.create_publisher(WheelFeedback, 'wheel_feedback', 10)
        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(WheelCommand, 'wheel_cmd', self._on_wheel_cmd, 10)

        rate_hz = self.get_parameter('update_rate_hz').value
        self._dt = 1.0 / rate_hz
        self.create_timer(self._dt, self._on_tick)

        self.get_logger().info('rp1_sim ready: simulating wheel dynamics + odometry, no CAN')

    def _on_wheel_cmd(self, msg: WheelCommand) -> None:
        self._last_cmd_time = self.get_clock().now()
        self._latest_velocity = list(msg.velocity)

    def _on_tick(self) -> None:
        velocity = self._latest_velocity
        if self._last_cmd_time is not None:
            stale = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
            if stale > self._cmd_timeout:
                velocity = [0.0, 0.0, 0.0, 0.0]
        else:
            velocity = [0.0, 0.0, 0.0, 0.0]

        left = (velocity[WheelCommand.WHEEL_FRONT_LEFT]
                + velocity[WheelCommand.WHEEL_REAR_LEFT]) / 2.0 * self._max_wheel_speed
        right = (velocity[WheelCommand.WHEEL_FRONT_RIGHT]
                 + velocity[WheelCommand.WHEEL_REAR_RIGHT]) / 2.0 * self._max_wheel_speed

        v = (left + right) / 2.0
        w = (right - left) / self._track_width

        self._x += v * math.cos(self._yaw) * self._dt
        self._y += v * math.sin(self._yaw) * self._dt
        self._yaw += w * self._dt

        now = self.get_clock().now().to_msg()
        self._publish_odom(now, v, w)
        if self._publish_tf:
            self._broadcast_tf(now)
        self._publish_wheel_feedback(now, velocity)

    def _publish_odom(self, stamp, v: float, w: float) -> None:
        qz = math.sin(self._yaw / 2.0)
        qw = math.cos(self._yaw / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame_id
        odom.child_frame_id = self._base_frame_id
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self._odom_pub.publish(odom)

    def _broadcast_tf(self, stamp) -> None:
        qz = math.sin(self._yaw / 2.0)
        qw = math.cos(self._yaw / 2.0)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self._odom_frame_id
        t.child_frame_id = self._base_frame_id
        t.transform.translation.x = self._x
        t.transform.translation.y = self._y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(t)

    def _publish_wheel_feedback(self, stamp, velocity) -> None:
        for index in (WheelCommand.WHEEL_FRONT_LEFT, WheelCommand.WHEEL_FRONT_RIGHT,
                      WheelCommand.WHEEL_REAR_LEFT, WheelCommand.WHEEL_REAR_RIGHT):
            wheel_speed = velocity[index] * self._max_wheel_speed
            rpm = wheel_speed / (2.0 * math.pi * self._wheel_radius) * 60.0

            msg = WheelFeedback()
            msg.header.stamp = stamp
            msg.wheel_index = index
            msg.rpm = rpm
            msg.voltage = NOMINAL_VOLTAGE
            msg.current = abs(velocity[index]) * CURRENT_PER_UNIT_VELOCITY
            self._feedback_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
