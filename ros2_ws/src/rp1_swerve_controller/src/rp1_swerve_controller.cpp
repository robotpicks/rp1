#include "rp1_swerve_controller/rp1_swerve_controller.hpp"

#include <algorithm>
#include <cmath>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace rp1_swerve_controller
{

namespace
{
// Wraps an angle to (-pi, pi].
double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}
}  // namespace

controller_interface::CallbackReturn RP1SwerveController::on_init()
{
  auto node = get_node();
  drive_joint_names_ = node->declare_parameter<std::vector<std::string>>(
    "drive_joints", std::vector<std::string>{});
  steering_joint_names_ = node->declare_parameter<std::vector<std::string>>(
    "steering_joints", std::vector<std::string>{});

  if (drive_joint_names_.size() != NUM_CORNERS || steering_joint_names_.size() != NUM_CORNERS)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "drive_joints and steering_joints must each list exactly %zu joint names "
      "(front_left, front_right, rear_left, rear_right) -- got %zu and %zu",
      NUM_CORNERS, drive_joint_names_.size(), steering_joint_names_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  half_wheelbase_ = node->declare_parameter<double>("half_wheelbase", half_wheelbase_);
  half_track_ = node->declare_parameter<double>("half_track", half_track_);
  wheel_radius_ = node->declare_parameter<double>("wheel_radius", wheel_radius_);
  if (half_wheelbase_ <= 0.0 || half_track_ <= 0.0 || wheel_radius_ <= 0.0)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "half_wheelbase (%f), half_track (%f), and wheel_radius (%f) must all be positive",
      half_wheelbase_, half_track_, wheel_radius_);
    return controller_interface::CallbackReturn::ERROR;
  }

  odom_frame_id_ = node->declare_parameter<std::string>("odom_frame_id", odom_frame_id_);
  base_frame_id_ = node->declare_parameter<std::string>("base_frame_id", base_frame_id_);
  enable_odom_tf_ = node->declare_parameter<bool>("enable_odom_tf", enable_odom_tf_);

  // CornerIndex order: front_left, front_right, rear_left, rear_right. X forward, Y left
  // (REP-103) -- matches rp1-specs/mechanical_spec.md's steering-axis table.
  corner_position_[FRONT_LEFT] = {half_wheelbase_, half_track_};
  corner_position_[FRONT_RIGHT] = {half_wheelbase_, -half_track_};
  corner_position_[REAR_LEFT] = {-half_wheelbase_, half_track_};
  corner_position_[REAR_RIGHT] = {-half_wheelbase_, -half_track_};

  last_steering_angle_.fill(0.0);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
RP1SwerveController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & name : drive_joint_names_)
  {
    config.names.push_back(name + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  for (const auto & name : steering_joint_names_)
  {
    config.names.push_back(name + "/" + hardware_interface::HW_IF_POSITION);
  }
  return config;
}

controller_interface::InterfaceConfiguration
RP1SwerveController::state_interface_configuration() const
{
  // Drive velocity + steering position state, for odometry (compute_body_twist()) -- the IK
  // itself (compute_corner_commands()) is still open-loop from cmd_vel. Does not yet read the
  // 0/90-degree home-switch or seek_home interfaces (rp1-specs/requirements.md's mode state
  // machine, not implemented here).
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & name : drive_joint_names_)
  {
    config.names.push_back(name + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  for (const auto & name : steering_joint_names_)
  {
    config.names.push_back(name + "/" + hardware_interface::HW_IF_POSITION);
  }
  return config;
}

controller_interface::CallbackReturn RP1SwerveController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  input_cmd_vel_.set(nullptr);
  cmd_vel_subscriber_ = get_node()->create_subscription<TwistStamped>(
    "~/cmd_vel", rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<TwistStamped> msg)
    { input_cmd_vel_.set([&msg](std::shared_ptr<TwistStamped> & value) { value = msg; }); });

  odom_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
    "~/odom", rclcpp::SystemDefaultsQoS());
  realtime_odom_publisher_ = std::make_unique<OdomPublisher>(odom_publisher_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(get_node());

  odom_x_ = 0.0;
  odom_y_ = 0.0;
  odom_yaw_ = 0.0;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RP1SwerveController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  drive_velocity_command_.clear();
  steering_position_command_.clear();
  for (const auto & name : drive_joint_names_)
  {
    auto it = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&name](const auto & iface)
      {
        return iface.get_prefix_name() == name &&
               iface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
      });
    if (it == command_interfaces_.end())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Could not find velocity command interface for '%s'",
        name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    drive_velocity_command_.emplace_back(*it);
  }
  for (const auto & name : steering_joint_names_)
  {
    auto it = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&name](const auto & iface)
      {
        return iface.get_prefix_name() == name &&
               iface.get_interface_name() == hardware_interface::HW_IF_POSITION;
      });
    if (it == command_interfaces_.end())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Could not find position command interface for '%s'",
        name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    steering_position_command_.emplace_back(*it);
  }

  drive_velocity_state_.clear();
  steering_position_state_.clear();
  for (const auto & name : drive_joint_names_)
  {
    auto it = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(),
      [&name](const auto & iface)
      {
        return iface.get_prefix_name() == name &&
               iface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
      });
    if (it == state_interfaces_.end())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Could not find velocity state interface for '%s'",
        name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    drive_velocity_state_.emplace_back(*it);
  }
  for (const auto & name : steering_joint_names_)
  {
    auto it = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(),
      [&name](const auto & iface)
      {
        return iface.get_prefix_name() == name &&
               iface.get_interface_name() == hardware_interface::HW_IF_POSITION;
      });
    if (it == state_interfaces_.end())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Could not find position state interface for '%s'",
        name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    steering_position_state_.emplace_back(*it);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RP1SwerveController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  drive_velocity_command_.clear();
  steering_position_command_.clear();
  drive_velocity_state_.clear();
  steering_position_state_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

void RP1SwerveController::compute_corner_commands(
  const TwistStamped & cmd_vel, std::array<double, NUM_CORNERS> & wheel_velocity,
  std::array<double, NUM_CORNERS> & steering_angle)
{
  const double vx = cmd_vel.twist.linear.x;
  const double vy = cmd_vel.twist.linear.y;
  const double wz = cmd_vel.twist.angular.z;

  for (std::size_t i = 0; i < NUM_CORNERS; ++i)
  {
    const auto [x_i, y_i] = corner_position_[i];

    // Rigid-body twist at this corner: v_wheel = v_body + wz x r_i, r_i = (x_i, y_i).
    const double vwx = vx - wz * y_i;
    const double vwy = vy + wz * x_i;

    double speed = std::hypot(vwx, vwy);
    double angle = speed > 1e-9 ? std::atan2(vwy, vwx) : last_steering_angle_[i];

    // Angle-flip optimization: a continuous joint can reach the same physical wheel direction by
    // rotating to `angle` and driving forward, or to `angle + pi` and driving in reverse --
    // whichever is the shorter rotation from where the wheel is now. Compare against the last
    // *unwrapped* commanded angle, not a re-wrapped one, so this stays correct across repeated
    // cycles rather than only ever comparing to a value in (-pi, pi].
    double delta = normalize_angle(angle - last_steering_angle_[i]);
    if (std::abs(delta) > M_PI_2)
    {
      angle = normalize_angle(angle + M_PI);
      speed = -speed;
      delta = normalize_angle(angle - last_steering_angle_[i]);
    }

    const double unwrapped_angle = last_steering_angle_[i] + delta;
    last_steering_angle_[i] = unwrapped_angle;

    wheel_velocity[i] = speed;
    steering_angle[i] = unwrapped_angle;
  }
}

void RP1SwerveController::compute_body_twist(double & vx, double & vy, double & wz) const
{
  // Least-squares fit of (vx, vy, wz) to the per-corner wheel velocity vectors, same formulation
  // as rp1_sim/swerve_sim_node.py's _solve_body_twist(): each corner contributes two equations,
  // v_wheel = v_body + wz x r_i, r_i = (x_i, y_i):
  //   vx - wz*y_i = vwx_i
  //   vy + wz*x_i = vwy_i
  // Normal equations for this (8 equations, 3 unknowns) reduce to a 3x3 linear system whose
  // off-diagonal terms are sum(x_i) and sum(y_i) -- both exactly zero for corner_position_'s
  // always-rectangular geometry (the four +/-half_wheelbase, +/-half_track combinations sum to
  // zero on each axis), so the system is exactly diagonal and decouples into three independent
  // averages. NOT valid if corner_position_ were ever set asymmetrically.
  double sum_vwx = 0.0;
  double sum_vwy = 0.0;
  double sum_wz_num = 0.0;
  double sum_wz_den = 0.0;

  for (std::size_t i = 0; i < NUM_CORNERS; ++i)
  {
    const auto [x_i, y_i] = corner_position_[i];
    const double wheel_speed =
      wheel_radius_ * drive_velocity_state_[i].get().get_optional().value_or(0.0);
    const double angle = steering_position_state_[i].get().get_optional().value_or(0.0);
    const double vwx = wheel_speed * std::cos(angle);
    const double vwy = wheel_speed * std::sin(angle);

    sum_vwx += vwx;
    sum_vwy += vwy;
    sum_wz_num += -y_i * vwx + x_i * vwy;
    sum_wz_den += x_i * x_i + y_i * y_i;
  }

  vx = sum_vwx / static_cast<double>(NUM_CORNERS);
  vy = sum_vwy / static_cast<double>(NUM_CORNERS);
  wz = sum_wz_den > 1e-9 ? sum_wz_num / sum_wz_den : 0.0;
}

controller_interface::return_type RP1SwerveController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  std::shared_ptr<TwistStamped> cmd_vel;
  input_cmd_vel_.get(
    [&cmd_vel](const std::shared_ptr<TwistStamped> & value) { cmd_vel = value; });

  std::array<double, NUM_CORNERS> wheel_velocity{};
  std::array<double, NUM_CORNERS> steering_angle{};
  if (cmd_vel)
  {
    compute_corner_commands(*cmd_vel, wheel_velocity, steering_angle);
  }

  for (std::size_t i = 0; i < NUM_CORNERS; ++i)
  {
    // set_value() is [[nodiscard]] on this ROS 2 release (it can fail under lock contention);
    // nothing meaningful to do differently on failure here yet, but don't silently drop it.
    if (!drive_velocity_command_[i].get().set_value(wheel_velocity[i]))
    {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Failed to set drive velocity command for corner %zu", i);
    }
    if (!steering_position_command_[i].get().set_value(steering_angle[i]))
    {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Failed to set steering position command for corner %zu", i);
    }
  }

  double vx = 0.0;
  double vy = 0.0;
  double wz = 0.0;
  compute_body_twist(vx, vy, wz);

  const double dt = period.seconds();
  // Integrate in the world/odom frame: rotate the body-frame twist by the current yaw before
  // accumulating position, same convention as rp1_sim/swerve_sim_node.py.
  odom_x_ += (vx * std::cos(odom_yaw_) - vy * std::sin(odom_yaw_)) * dt;
  odom_y_ += (vx * std::sin(odom_yaw_) + vy * std::cos(odom_yaw_)) * dt;
  odom_yaw_ = normalize_angle(odom_yaw_ + wz * dt);

  const double half_yaw = odom_yaw_ / 2.0;
  const double qz = std::sin(half_yaw);
  const double qw = std::cos(half_yaw);

  if (realtime_odom_publisher_)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = time;
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id = base_frame_id_;
    odom.pose.pose.position.x = odom_x_;
    odom.pose.pose.position.y = odom_y_;
    odom.pose.pose.orientation.z = qz;
    odom.pose.pose.orientation.w = qw;
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = wz;
    // try_publish() drops the message rather than blocking if the background publishing thread
    // is busy -- fine for odometry at a fixed control-loop rate, matching this ROS release's
    // realtime_tools::RealtimePublisher API (replaces the older trylock()/msg_/
    // unlockAndPublish() pattern other controllers in this codebase's ROS distro used).
    realtime_odom_publisher_->try_publish(odom);
  }

  if (enable_odom_tf_ && tf_broadcaster_)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = time;
    tf.header.frame_id = odom_frame_id_;
    tf.child_frame_id = base_frame_id_;
    tf.transform.translation.x = odom_x_;
    tf.transform.translation.y = odom_y_;
    tf.transform.rotation.z = qz;
    tf.transform.rotation.w = qw;
    tf_broadcaster_->sendTransform(tf);
  }

  return controller_interface::return_type::OK;
}

}  // namespace rp1_swerve_controller

PLUGINLIB_EXPORT_CLASS(
  rp1_swerve_controller::RP1SwerveController, controller_interface::ControllerInterface)
