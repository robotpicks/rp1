#include "rp1_swerve_controller/rp1_swerve_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

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

  steering_home_sensor_names_ = node->declare_parameter<std::vector<std::string>>(
    "steering_home_sensors", steering_home_sensor_names_);
  if (steering_home_sensor_names_.size() != NUM_CORNERS)
  {
    RCLCPP_ERROR(
      node->get_logger(), "steering_home_sensors must list exactly %zu gpio names -- got %zu",
      NUM_CORNERS, steering_home_sensor_names_.size());
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

  mode_switch_stopped_tolerance_ = node->declare_parameter<double>(
    "mode_switch_stopped_tolerance", mode_switch_stopped_tolerance_);
  if (mode_switch_stopped_tolerance_ < 0.0)
  {
    RCLCPP_ERROR(
      node->get_logger(), "mode_switch_stopped_tolerance (%f) must not be negative",
      mode_switch_stopped_tolerance_);
    return controller_interface::CallbackReturn::ERROR;
  }

  // CornerIndex order: front_left, front_right, rear_left, rear_right. X forward, Y left
  // (REP-103) -- matches rp1-specs/mechanical_spec.md's steering-axis table.
  corner_position_[FRONT_LEFT] = {half_wheelbase_, half_track_};
  corner_position_[FRONT_RIGHT] = {half_wheelbase_, -half_track_};
  corner_position_[REAR_LEFT] = {-half_wheelbase_, half_track_};
  corner_position_[REAR_RIGHT] = {-half_wheelbase_, -half_track_};

  // Which 2 corners free-steer in TWO_WHEEL mode -- any 2, not fixed to a front/rear pair (see
  // the header). Default is the front pair (a conventional front-steered vehicle), but any
  // combination is valid: one front + one rear, both on one side, or diagonal corners.
  const auto two_wheel_steered_names = node->declare_parameter<std::vector<std::string>>(
    "two_wheel_steered_corners", std::vector<std::string>{"front_left", "front_right"});
  if (two_wheel_steered_names.size() != 2)
  {
    RCLCPP_ERROR(
      node->get_logger(), "two_wheel_steered_corners must list exactly 2 corner names -- got %zu",
      two_wheel_steered_names.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  const std::array<std::pair<const char *, CornerIndex>, NUM_CORNERS> corner_name_lookup{{
    {"front_left", FRONT_LEFT},
    {"front_right", FRONT_RIGHT},
    {"rear_left", REAR_LEFT},
    {"rear_right", REAR_RIGHT},
  }};
  two_wheel_steered_.fill(false);
  for (const auto & name : two_wheel_steered_names)
  {
    auto it = std::find_if(
      corner_name_lookup.begin(), corner_name_lookup.end(),
      [&name](const auto & entry) { return name == entry.first; });
    if (it == corner_name_lookup.end())
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "two_wheel_steered_corners entry '%s' is not one of front_left, front_right, rear_left, "
        "rear_right",
        name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    two_wheel_steered_[it->second] = true;
  }

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
    config.names.push_back(name + "/seek_home");
  }
  return config;
}

controller_interface::InterfaceConfiguration
RP1SwerveController::state_interface_configuration() const
{
  // Drive velocity + steering position state, for odometry (compute_body_twist()) -- the IK
  // itself (compute_corner_commands()) is still open-loop from cmd_vel. Also reads the 0/90-
  // degree home-switch gpio state (steering_home_sensor_names_) to gate locked-mode transitions
  // -- see compute_corner_commands().
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
  for (const auto & name : steering_home_sensor_names_)
  {
    config.names.push_back(name + "/home_0deg");
    config.names.push_back(name + "/home_90deg");
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

  mode_.store(static_cast<uint8_t>(SwerveMode::FULL_SWERVE));
  active_mode_ = SwerveMode::FULL_SWERVE;
  mode_subscriber_ = get_node()->create_subscription<std_msgs::msg::UInt8>(
    "~/mode", rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<std_msgs::msg::UInt8> msg) { mode_.store(msg->data); });

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
  steering_seek_home_command_.clear();
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

    auto seek_home_it = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&name](const auto & iface)
      { return iface.get_prefix_name() == name && iface.get_interface_name() == "seek_home"; });
    if (seek_home_it == command_interfaces_.end())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Could not find seek_home command interface for '%s'",
        name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    steering_seek_home_command_.emplace_back(*seek_home_it);
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

  steering_home_0deg_state_.clear();
  steering_home_90deg_state_.clear();
  for (const auto & name : steering_home_sensor_names_)
  {
    auto home_0deg_it = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(),
      [&name](const auto & iface)
      { return iface.get_prefix_name() == name && iface.get_interface_name() == "home_0deg"; });
    if (home_0deg_it == state_interfaces_.end())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Could not find home_0deg state interface for '%s'",
        name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    steering_home_0deg_state_.emplace_back(*home_0deg_it);

    auto home_90deg_it = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(),
      [&name](const auto & iface)
      { return iface.get_prefix_name() == name && iface.get_interface_name() == "home_90deg"; });
    if (home_90deg_it == state_interfaces_.end())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Could not find home_90deg state interface for '%s'",
        name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    steering_home_90deg_state_.emplace_back(*home_90deg_it);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RP1SwerveController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  drive_velocity_command_.clear();
  steering_position_command_.clear();
  steering_seek_home_command_.clear();
  drive_velocity_state_.clear();
  steering_position_state_.clear();
  steering_home_0deg_state_.clear();
  steering_home_90deg_state_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

SwerveMode RP1SwerveController::validate_mode(uint8_t raw)
{
  auto mode = static_cast<SwerveMode>(raw);
  if (mode != SwerveMode::FULL_SWERVE && mode != SwerveMode::LOCKED_0 &&
      mode != SwerveMode::LOCKED_90 && mode != SwerveMode::TWO_WHEEL)
  {
    // Unrecognized value on ~/mode (stray publish, uninitialized field, a future mode this
    // build doesn't know about) -- fail safe to the one mode that's always geometrically valid
    // rather than silently doing nothing or crashing on the switch below.
    return SwerveMode::FULL_SWERVE;
  }
  return mode;
}

void RP1SwerveController::compute_corner_commands(
  SwerveMode mode, const TwistStamped & cmd_vel, std::array<double, NUM_CORNERS> & wheel_velocity,
  std::array<double, NUM_CORNERS> & steering_angle,
  std::array<double, NUM_CORNERS> & seek_home_command)
{
  const double vx = cmd_vel.twist.linear.x;
  const double vy = cmd_vel.twist.linear.y;
  const double wz = cmd_vel.twist.angular.z;

  // First pass: which corners are locked to a fixed reference angle this mode (only
  // LOCKED_0/LOCKED_90/TWO_WHEEL's non-free corners), and -- for those -- whether the physical
  // 0/90-degree reference is actually confirmed yet via home_0deg/home_90deg. Read live every
  // cycle rather than tracked as one-shot "have we ever homed" state: a corner that later moves
  // off the reference (e.g. driven in FULL_SWERVE) naturally stops reporting it, so the next
  // time a locked mode needs that corner it re-gates automatically.
  std::array<bool, NUM_CORNERS> locked{};
  std::array<double, NUM_CORNERS> locked_angle{};
  bool any_pending_home = false;

  for (std::size_t i = 0; i < NUM_CORNERS; ++i)
  {
    bool this_locked = false;
    double this_angle = 0.0;
    bool target_is_90deg = false;
    switch (mode)
    {
      case SwerveMode::LOCKED_0:
        this_locked = true;
        this_angle = 0.0;
        break;
      case SwerveMode::LOCKED_90:
        this_locked = true;
        this_angle = M_PI_2;
        target_is_90deg = true;
        break;
      case SwerveMode::TWO_WHEEL:
        if (!two_wheel_steered_[i])
        {
          this_locked = true;
          this_angle = 0.0;
        }
        break;
      case SwerveMode::FULL_SWERVE:
        break;
    }
    locked[i] = this_locked;
    locked_angle[i] = this_angle;

    if (!this_locked)
    {
      seek_home_command[i] = std::numeric_limits<double>::quiet_NaN();
      continue;
    }

    const bool confirmed =
      (target_is_90deg ? steering_home_90deg_state_[i] : steering_home_0deg_state_[i])
        .get()
        .get_optional()
        .value_or(0.0) > 0.5;
    if (confirmed)
    {
      seek_home_command[i] = std::numeric_limits<double>::quiet_NaN();
    }
    else
    {
      seek_home_command[i] = target_is_90deg ? 1.0 : 0.0;
      any_pending_home = true;
    }
  }

  for (std::size_t i = 0; i < NUM_CORNERS; ++i)
  {
    const auto [x_i, y_i] = corner_position_[i];

    // Rigid-body twist at this corner: v_wheel = v_body + wz x r_i, r_i = (x_i, y_i).
    const double vwx = vx - wz * y_i;
    const double vwy = vy + wz * x_i;

    double speed;
    double angle;

    if (locked[i])
    {
      // Project the same rigid-body-twist vector onto the fixed direction instead of solving
      // for a free angle -- exactly the standard skid-steer differential-drive formula when
      // locked_angle is 0 (vwx = vx - wz*y_i, matching diff_drive_controller's per-side
      // v = vx -/+ wz*track/2 convention for left/right at y_i = +/-half_track). No angle-flip
      // optimization here: a locked wheel should stay visually fixed at locked_angle, not flip
      // to the opposite angle with reversed speed even though that's motion-equivalent -- the
      // whole point of "locked" is a known, fixed wheel orientation.
      speed = vwx * std::cos(locked_angle[i]) + vwy * std::sin(locked_angle[i]);

      // Still take the shortest unwrapped path to locked_angle (not the flip-optimized path),
      // so a corner switching INTO a locked mode from some arbitrary free-swerve angle doesn't
      // command a multi-turn spin, and so last_steering_angle_ keeps meaning "the unwrapped
      // angle actually commanded" for whichever mode runs next.
      const double delta = normalize_angle(locked_angle[i] - last_steering_angle_[i]);
      angle = last_steering_angle_[i] + delta;
    }
    else
    {
      speed = std::hypot(vwx, vwy);
      angle = speed > 1e-9 ? std::atan2(vwy, vwx) : last_steering_angle_[i];

      // Angle-flip optimization: a continuous joint can reach the same physical wheel direction
      // by rotating to `angle` and driving forward, or to `angle + pi` and driving in reverse --
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
      angle = last_steering_angle_[i] + delta;
    }

    if (any_pending_home)
    {
      // Hold the whole chassis still -- not just this corner -- until every locked corner
      // confirms its physical reference: a mix of some corners already at 0/90 and others still
      // seeking would drag/scrub if the drive wheels moved. The steering position command above
      // still targets locked_angle as usual, so a locked corner visibly rotates in place to seek
      // its sensor (per rp1-specs/requirements.md) even though nothing translates yet.
      speed = 0.0;
    }

    last_steering_angle_[i] = angle;
    wheel_velocity[i] = speed;
    steering_angle[i] = angle;
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

  // A state interface that hasn't been reported yet (real hardware before its first telemetry
  // frame, or a bare test-harness interface with no initial_value) reads back as an ENGAGED
  // optional wrapping NaN, not an empty one -- get_optional().value_or(0.0) only substitutes for
  // the latter, so a NaN payload would otherwise poison every downstream sum. Treat both cases
  // the same way: no data yet contributes zero.
  auto state_or_zero = [](const hardware_interface::LoanedStateInterface & iface)
  {
    const double value = iface.get_optional().value_or(0.0);
    return std::isnan(value) ? 0.0 : value;
  };

  for (std::size_t i = 0; i < NUM_CORNERS; ++i)
  {
    const auto [x_i, y_i] = corner_position_[i];
    const double wheel_speed = wheel_radius_ * state_or_zero(drive_velocity_state_[i].get());
    const double angle = state_or_zero(steering_position_state_[i].get());
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

  // Body twist from real state feedback (not the command), computed once and reused both to
  // gate mode transitions below and to publish ~/odom further down.
  double vx = 0.0;
  double vy = 0.0;
  double wz = 0.0;
  compute_body_twist(vx, vy, wz);

  // Mode-switch safety (rp1-specs/requirements.md's "no ramping, no requirement to be stopped
  // first, no rejection of a switch mid-turn" gap): a requested mode only becomes the ACTIVE one
  // once the chassis is confirmed actually stopped, not just commanded to stop -- otherwise
  // compute_corner_commands() keeps running the previous mode's kinematics. This is a blanket
  // rule independent of the home_0deg/home_90deg gate inside compute_corner_commands() (which
  // only fires when a corner's target angle isn't yet confirmed); this one covers every mode
  // transition, including ones where no corner's target angle actually changes.
  const auto requested_mode = validate_mode(mode_.load());
  const bool is_stopped = std::abs(vx) < mode_switch_stopped_tolerance_ &&
                           std::abs(vy) < mode_switch_stopped_tolerance_ &&
                           std::abs(wz) < mode_switch_stopped_tolerance_;
  if (requested_mode != active_mode_ && is_stopped)
  {
    active_mode_ = requested_mode;
  }

  std::array<double, NUM_CORNERS> wheel_velocity{};
  std::array<double, NUM_CORNERS> steering_angle{};
  std::array<double, NUM_CORNERS> seek_home_command{};
  // Always compute, even before the first ~/cmd_vel arrives (a zero twist): homing needs to
  // start as soon as a locked mode is requested, not wait on motion commands that may never come
  // if the robot is meant to sit still while it homes.
  static const TwistStamped kZeroTwist{};
  compute_corner_commands(
    active_mode_, cmd_vel ? *cmd_vel : kZeroTwist, wheel_velocity, steering_angle,
    seek_home_command);

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
    if (!steering_seek_home_command_[i].get().set_value(seek_home_command[i]))
    {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Failed to set seek_home command for corner %zu", i);
    }
  }

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
