"""Pure-ROS2 swerve-drive simulator: no CAN, no DroneCAN, no VESCs.

Extends rp1_sim's skid-steer simulator (sim_bridge_node.py) to the full 4-wheel +
4-steering-actuator case. Subscribes `rp1_msgs/WheelCommand` (per-wheel drive speed, same as
the MVP) and `rp1_msgs/SteeringCommand` (per-wheel steering angle, radians -- see
docs/can_id_map.md's steering actuator_id = wheel index + 4 convention, which is a DroneCAN
addressing detail this node doesn't need to know about), integrates full swerve forward
kinematics via least-squares body-twist fitting, and publishes `rp1_msgs/WheelFeedback` +
`rp1_msgs/SteeringFeedback` + `nav_msgs/Odometry` + an `odom` -> `base_link` TF.

There is no swerve inverse-kinematics controller in rp1_control yet (see CLAUDE.md's
"Extending to full swerve" section) -- this node takes already-resolved per-wheel speed+angle
commands directly, the same way sim_bridge_node.py takes already-resolved skid-steer
WheelCommand. It's a stand-in for what the real robot would do given those commands, not a
stand-in for the not-yet-built inverse kinematics.

Steering actuator dynamics are a simple rate-limited slew toward the commanded angle (shortest
angular path), not a physical FOC/servo model -- good enough for plausible-looking telemetry.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rp1_msgs.msg import SteeringCommand, SteeringFeedback, WheelCommand, WheelFeedback
from tf2_ros import TransformBroadcaster

NOMINAL_VOLTAGE = 24.0
CURRENT_PER_UNIT_VELOCITY = 10.0

WHEEL_INDICES = (
    WheelCommand.WHEEL_FRONT_LEFT, WheelCommand.WHEEL_FRONT_RIGHT,
    WheelCommand.WHEEL_REAR_LEFT, WheelCommand.WHEEL_REAR_RIGHT,
)


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class SwerveSimNode(Node):

    def __init__(self):
        super().__init__('swerve_sim_node')

        self.declare_parameter('track_width', 0.5)
        self.declare_parameter('wheelbase', 0.5)
        self.declare_parameter('max_wheel_speed', 1.0)
        self.declare_parameter('max_steering_rate', 6.0)  # rad/s, actuator slew rate
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('update_rate_hz', 50.0)
        self.declare_parameter('cmd_timeout_sec', 0.2)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')

        self._max_wheel_speed = self.get_parameter('max_wheel_speed').value
        self._max_steering_rate = self.get_parameter('max_steering_rate').value
        self._wheel_radius = self.get_parameter('wheel_radius').value
        self._cmd_timeout = self.get_parameter('cmd_timeout_sec').value
        self._publish_tf = self.get_parameter('publish_tf').value
        self._odom_frame_id = self.get_parameter('odom_frame_id').value
        self._base_frame_id = self.get_parameter('base_frame_id').value

        track_width = self.get_parameter('track_width').value
        wheelbase = self.get_parameter('wheelbase').value
        # Wheel positions in the body frame (x forward, y left), indexed like WheelCommand.
        self._wheel_pos = {
            WheelCommand.WHEEL_FRONT_LEFT: (wheelbase / 2.0, track_width / 2.0),
            WheelCommand.WHEEL_FRONT_RIGHT: (wheelbase / 2.0, -track_width / 2.0),
            WheelCommand.WHEEL_REAR_LEFT: (-wheelbase / 2.0, track_width / 2.0),
            WheelCommand.WHEEL_REAR_RIGHT: (-wheelbase / 2.0, -track_width / 2.0),
        }

        self._latest_wheel_velocity = [0.0, 0.0, 0.0, 0.0]
        self._latest_steering_angle = [0.0, 0.0, 0.0, 0.0]
        self._steering_state = [0.0, 0.0, 0.0, 0.0]  # actuator's own slewed angle
        self._steering_velocity = [0.0, 0.0, 0.0, 0.0]  # rad/s, from the last slew step
        self._last_wheel_cmd_time = None
        self._last_steering_cmd_time = None
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        self._wheel_feedback_pub = self.create_publisher(WheelFeedback, 'wheel_feedback', 10)
        self._steering_feedback_pub = self.create_publisher(
            SteeringFeedback, 'steering_feedback', 10)
        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(WheelCommand, 'wheel_cmd', self._on_wheel_cmd, 10)
        self.create_subscription(SteeringCommand, 'steering_cmd', self._on_steering_cmd, 10)

        rate_hz = self.get_parameter('update_rate_hz').value
        self._dt = 1.0 / rate_hz
        self.create_timer(self._dt, self._on_tick)

        self.get_logger().info(
            'swerve_sim ready: simulating 4-wheel + 4-steering dynamics + odometry, no CAN')

    def _on_wheel_cmd(self, msg: WheelCommand) -> None:
        self._last_wheel_cmd_time = self.get_clock().now()
        self._latest_wheel_velocity = list(msg.velocity)

    def _on_steering_cmd(self, msg: SteeringCommand) -> None:
        self._last_steering_cmd_time = self.get_clock().now()
        self._latest_steering_angle = list(msg.angle)

    def _stale(self, last_time) -> bool:
        if last_time is None:
            return True
        age = (self.get_clock().now() - last_time).nanoseconds / 1e9
        return age > self._cmd_timeout

    def _on_tick(self) -> None:
        wheel_velocity = (
            [0.0, 0.0, 0.0, 0.0] if self._stale(self._last_wheel_cmd_time)
            else self._latest_wheel_velocity)
        steering_target = self._latest_steering_angle  # hold last commanded angle when stale

        self._step_steering(steering_target)
        vx, vy, wz = self._solve_body_twist(wheel_velocity, self._steering_state)

        self._x += (vx * math.cos(self._yaw) - vy * math.sin(self._yaw)) * self._dt
        self._y += (vx * math.sin(self._yaw) + vy * math.cos(self._yaw)) * self._dt
        self._yaw = _normalize_angle(self._yaw + wz * self._dt)

        now = self.get_clock().now().to_msg()
        self._publish_odom(now, vx, vy, wz)
        if self._publish_tf:
            self._broadcast_tf(now)
        self._publish_wheel_feedback(now, wheel_velocity)
        self._publish_steering_feedback(now)

    def _step_steering(self, target_angle) -> None:
        max_step = self._max_steering_rate * self._dt
        for index in WHEEL_INDICES:
            delta = _normalize_angle(target_angle[index] - self._steering_state[index])
            step = delta if abs(delta) <= max_step else math.copysign(max_step, delta)
            self._steering_state[index] = _normalize_angle(self._steering_state[index] + step)
            self._steering_velocity[index] = step / self._dt if self._dt > 0 else 0.0

    def _solve_body_twist(self, wheel_velocity, steering_angle):
        """Least-squares fit of body twist (vx, vy, wz) to the 4 wheels' commanded velocity
        vectors, given their steering angles and positions -- swerve forward kinematics."""
        rows = []
        targets = []
        for index in WHEEL_INDICES:
            x_i, y_i = self._wheel_pos[index]
            speed = wheel_velocity[index] * self._max_wheel_speed
            angle = steering_angle[index]
            vwx = speed * math.cos(angle)
            vwy = speed * math.sin(angle)
            rows.append([1.0, 0.0, -y_i])
            targets.append(vwx)
            rows.append([0.0, 1.0, x_i])
            targets.append(vwy)

        solution, _residuals, _rank, _sv = np.linalg.lstsq(
            np.array(rows), np.array(targets), rcond=None)
        return float(solution[0]), float(solution[1]), float(solution[2])

    def _publish_odom(self, stamp, vx: float, vy: float, wz: float) -> None:
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
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
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

    def _publish_wheel_feedback(self, stamp, wheel_velocity) -> None:
        for index in WHEEL_INDICES:
            wheel_speed = wheel_velocity[index] * self._max_wheel_speed
            rpm = wheel_speed / (2.0 * math.pi * self._wheel_radius) * 60.0

            msg = WheelFeedback()
            msg.header.stamp = stamp
            msg.wheel_index = index
            msg.rpm = rpm
            msg.voltage = NOMINAL_VOLTAGE
            msg.current = abs(wheel_velocity[index]) * CURRENT_PER_UNIT_VELOCITY
            self._wheel_feedback_pub.publish(msg)

    def _publish_steering_feedback(self, stamp) -> None:
        for index in WHEEL_INDICES:
            msg = SteeringFeedback()
            msg.header.stamp = stamp
            msg.wheel_index = index
            msg.angle = self._steering_state[index]
            msg.angular_velocity = self._steering_velocity[index]
            self._steering_feedback_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SwerveSimNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
