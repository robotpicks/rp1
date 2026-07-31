#ifndef RP1_SWERVE_CONTROLLER__RP1_SWERVE_CONTROLLER_HPP_
#define RP1_SWERVE_CONTROLLER__RP1_SWERVE_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

namespace rp1_swerve_controller
{

// Fixed corner order used throughout: matches docs/can_id_map.md's wheel index convention
// (front-left, front-right, rear-left, rear-right).
enum CornerIndex : std::size_t
{
  FRONT_LEFT = 0,
  FRONT_RIGHT = 1,
  REAR_LEFT = 2,
  REAR_RIGHT = 3,
};
static constexpr std::size_t NUM_CORNERS = 4;

// Operating modes from rp1-specs/requirements.md's "Why swerve, and what it needs to do".
// Selected via the ~/mode topic (std_msgs/UInt8) -- see compute_corner_commands().
enum class SwerveMode : uint8_t
{
  FULL_SWERVE = 0,  // all 4 corners independently steered (today's only behavior, unchanged)
  LOCKED_0 = 1,     // all 4 wheels straight ahead, skid-steer differential from vx+wz; vy dropped
  LOCKED_90 = 2,    // all 4 wheels perpendicular to travel (pure crab), from vy+wz; vx dropped
  TWO_WHEEL = 3,    // 2 corners free-steer (full IK), other 2 locked at 0 like LOCKED_0 --
                    // which 2 is the two_wheel_steered_corners parameter, not fixed in code
};

class RP1SwerveController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  using TwistStamped = geometry_msgs::msg::TwistStamped;

  // Joint names, one per corner in CornerIndex order. Declared as ROS parameters rather than
  // generate_parameter_library-codegen for this first skeleton -- revisit if/when the parameter
  // list grows past what's here (see the package README).
  std::vector<std::string> drive_joint_names_;
  std::vector<std::string> steering_joint_names_;
  // <gpio> prefix names carrying home_0deg/home_90deg, one per corner, CornerIndex order --
  // matches rp1_swerve.urdf's steering_sensors_* gpio blocks. A separate parameter from
  // steering_joint_names_ (not derived from it) since these are gpio interfaces on their own
  // block, not the steering joint's own state interfaces. Default matches the current URDF's
  // naming convention.
  std::vector<std::string> steering_home_sensor_names_{
    "steering_sensors_front_left", "steering_sensors_front_right", "steering_sensors_rear_left",
    "steering_sensors_rear_right"};
  // False for a hardware component with no physical concept of a homing seek or a proximity-
  // sensor gpio -- gz_ros2_control's GazeboSimSystem, which only backs standard joint interfaces
  // (position/velocity/effort) with a real Gazebo entity. Must be known at
  // command_interface_configuration()/state_interface_configuration() time (declared in
  // on_init(), can't be discovered later): controller_manager's resource manager refuses to
  // activate a controller that DECLARES an interface no hardware component actually exports, so
  // seek_home/home_0deg/home_90deg can't just be requested-then-tolerated-if-missing the way
  // on_activate() tolerates a wrong steering_home_sensors name -- they must never be requested
  // at all when this is false. Set via the steering_home_sensors_available parameter (default
  // true, matching mock/real DroneCAN hardware, both of which do export these).
  bool steering_home_sensors_available_ = true;

  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    drive_velocity_command_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    steering_position_command_;
  // Edge-triggered per vesc_dronecan_driver's convention (write 0.0/1.0 to request a seek, NaN
  // means "no active request this cycle") -- see compute_corner_commands(). EMPTY (not
  // NUM_CORNERS-sized) when steering_home_sensors_available_ is false -- a hardware component
  // with no physical concept of a homing seek (gz_ros2_control's GazeboSimSystem) simply isn't
  // asked to export this per-joint custom interface at all (see that flag's header comment for
  // why it can't just be requested-then-tolerated-if-missing).
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    steering_seek_home_command_;

  // State feedback, read each update() to compute odometry -- NOT used by
  // compute_corner_commands() (the IK is open-loop from cmd_vel, same as before). On mock
  // hardware this is just last cycle's command looped back; on the real DroneCAN hardware it
  // would be the VESCs' actual reported speed/position.
  std::vector<std::reference_wrapper<const hardware_interface::LoanedStateInterface>>
    drive_velocity_state_;
  std::vector<std::reference_wrapper<const hardware_interface::LoanedStateInterface>>
    steering_position_state_;
  // 0/90-degree home-switch state, one pair of <gpio> interfaces per corner (rp1_swerve.urdf's
  // steering_sensors_* blocks -- named via the steering_home_sensors parameter, NOT the steering
  // joint name, since these live on a separate gpio block). Read live every cycle to gate locked-
  // mode transitions -- see compute_corner_commands()'s header comment for why no separate
  // "have we ever homed" bookkeeping is needed beyond this. EMPTY when
  // steering_home_sensors_available_ is false, same reasoning as steering_seek_home_command_
  // above -- every corner is then treated as permanently unconfirmed (see
  // compute_corner_commands()), the same practical outcome mock hardware already has today (the
  // interface exists there, but nothing ever sets it true).
  std::vector<std::reference_wrapper<const hardware_interface::LoanedStateInterface>>
    steering_home_0deg_state_;
  std::vector<std::reference_wrapper<const hardware_interface::LoanedStateInterface>>
    steering_home_90deg_state_;

  rclcpp::Subscription<TwistStamped>::SharedPtr cmd_vel_subscriber_;
  realtime_tools::RealtimeThreadSafeBox<std::shared_ptr<TwistStamped>> input_cmd_vel_{nullptr};

  // Mode select, ~/mode (std_msgs/UInt8, SwerveMode's underlying value). A trivially-copyable
  // POD, so a plain atomic is enough -- no need for the RealtimeThreadSafeBox machinery
  // input_cmd_vel_ uses for a non-trivial shared_ptr payload. Unrecognized values fall back to
  // FULL_SWERVE (see validate_mode()) rather than erroring, since a mode topic is easy to publish
  // a stray/uninitialized value on. This is the RAW requested mode -- update() only ever commits
  // it to active_mode_ once the robot is confirmed stopped; compute_corner_commands() acts on
  // active_mode_, never mode_ directly.
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mode_subscriber_;
  std::atomic<uint8_t> mode_{static_cast<uint8_t>(SwerveMode::FULL_SWERVE)};

  // The mode compute_corner_commands() actually acts on, as opposed to mode_ (the raw requested
  // value). update() only advances active_mode_ to the current requested mode once the robot's
  // real (state-feedback-derived) body twist reads below mode_switch_stopped_tolerance_ on all
  // 3 axes -- rp1-specs/requirements.md's "no ramping, no requirement to be stopped first, no
  // rejection of a switch mid-turn" gap. Complements, rather than replaces, the home_0deg/
  // home_90deg gate in compute_corner_commands(): that gate only fires when a corner's target
  // angle actually changes and isn't yet confirmed there; this one is a blanket "don't reshuffle
  // which corners are locked, or to what, while the chassis is actually moving" rule that covers
  // every mode transition regardless of which corners are affected. Reset to FULL_SWERVE in
  // on_configure(), matching mode_'s own reset.
  SwerveMode active_mode_ = SwerveMode::FULL_SWERVE;
  double mode_switch_stopped_tolerance_ = 0.02;

  // Which 2 corners free-steer in TWO_WHEEL mode -- any 2 of the 4, not fixed to a front/rear
  // pair. Set via the two_wheel_steered_corners parameter (exactly 2 of "front_left",
  // "front_right", "rear_left", "rear_right"; default is the front pair, matching a
  // conventional front-steered vehicle, but e.g. one front + one rear, or the two on one side,
  // are equally valid choices this parameter supports). true = free-steers, false = locked at
  // 0 deg like LOCKED_0. Computed once in on_init(), read (not mutated) in
  // compute_corner_commands().
  std::array<bool, NUM_CORNERS> two_wheel_steered_{};

  // Steering-axis geometry, half-dimensions in metres (base_link frame, X forward/Y left) --
  // defaults match the CAD steering-axis positions in rp1-specs/mechanical_spec.md §3.1
  // (wheelbase 0.8 m -> half 0.4; steering-axis track 1.28 m -> half 0.64), NOT the drive-joint
  // positions (which differ by the ~0.071 m scrub radius, §3.2) -- swerve IK pivots each wheel
  // about its steering axis, so that's the geometry that matters here. Overridable via the
  // half_wheelbase/half_track parameters since these are CAD-unmeasured numbers.
  double half_wheelbase_ = 0.4;
  double half_track_ = 0.64;
  // Rolling radius, metres -- converts a drive joint's reported angular velocity (rad/s) into
  // linear wheel speed (m/s) for odometry. Default matches rp1_drive.urdf's wheel collision
  // cylinder / rp1-specs/mechanical_spec.md §3.3.1 (CAD-nominal, not a measured/loaded radius).
  double wheel_radius_ = 0.2154;

  std::string odom_frame_id_ = "odom";
  std::string base_frame_id_ = "base_link";
  bool enable_odom_tf_ = true;

  // Per-corner (x, y) in CornerIndex order, computed from half_wheelbase_/half_track_ in
  // on_configure().
  std::array<std::pair<double, double>, NUM_CORNERS> corner_position_{};

  // Last commanded steering angle per corner, radians, UNWRAPPED (not clamped to [-pi, pi]) --
  // continuous joints have no wraparound, and the angle-flip optimization in
  // compute_corner_commands() needs true continuity across cycles to pick the shorter rotation
  // and to avoid the commanded angle jumping by 2*pi at the wrap boundary.
  std::array<double, NUM_CORNERS> last_steering_angle_{};

  // Integrated pose, odom frame.
  double odom_x_ = 0.0;
  double odom_y_ = 0.0;
  double odom_yaw_ = 0.0;

  using OdomPublisher = realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  std::unique_ptr<OdomPublisher> realtime_odom_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

private:
  // Swerve forward kinematics: per-corner wheel speed + steering angle state -> body twist
  // (vx, vy, wz). Exact least-squares solution for this controller's always-rectangular corner
  // geometry (corner_position_ is always the 4 sign combinations of +/-half_wheelbase,
  // +/-half_track, which makes the normal equations diagonal -- see the .cpp for the derivation)
  // -- NOT a generic solver, would need revisiting if corner_position_ ever became asymmetric.
  void compute_body_twist(double & vx, double & vy, double & wz) const;

  // Validates a raw ~/mode value, falling back to FULL_SWERVE for anything not one of the 4
  // known SwerveMode values (a stray publish/uninitialized field on ~/mode shouldn't error or
  // silently do nothing -- FULL_SWERVE is the one mode that's always geometrically valid).
  static SwerveMode validate_mode(uint8_t raw);

  // Swerve inverse kinematics: (vx, vy, wz) -> per-corner wheel speed + steering angle, per
  // `mode` (the ALREADY-validated, already mode-switch-gated active_mode_ -- see update() and
  // active_mode_'s header comment; this function never reads mode_ itself). FULL_SWERVE:
  // standard rigid-body-twist decomposition per corner, plus the angle-flip optimization (rotate
  // the wheel <= 90 degrees by allowing negative speed instead of always rotating to the literal
  // computed angle) against last_steering_angle_ so small cmd_vel changes don't spin a wheel 180
  // degrees when reversing direction would be shorter. LOCKED_0/LOCKED_90/TWO_WHEEL: affected
  // corners are held at a fixed angle (no flip optimization -- a "locked" wheel should stay
  // visually fixed, not flip to the opposite angle with reversed speed even though that's
  // motion-equivalent) and driven by projecting the same rigid-body-twist vector onto that fixed
  // direction, which is exactly the standard skid-steer differential-drive formula when the
  // fixed angle is 0. Mutates last_steering_angle_, so not const.
  //
  // Homing gate (rp1-specs/requirements.md's "Transitioning between the two locked
  // configurations..." note): a corner locked to a fixed angle this mode is only trusted once
  // its home_0deg/home_90deg gpio state confirms it's physically there -- read live every cycle,
  // no separate "have we ever homed" bookkeeping needed, since a corner that later drives away
  // from the reference (e.g. back in FULL_SWERVE) naturally stops reporting it and re-gates the
  // next time a locked mode needs it. While ANY locked corner isn't yet confirmed, every drive
  // wheel's speed is held at zero (the chassis doesn't translate while a corner's reference is
  // unconfirmed) and seek_home_command carries 0.0/1.0 for that corner (NaN = no active
  // request, matching vesc_dronecan_driver's edge-triggered convention) -- the corner's steering
  // position command still targets the locked angle as usual, so it can visibly seek in place.
  void compute_corner_commands(
    SwerveMode mode, const TwistStamped & cmd_vel, std::array<double, NUM_CORNERS> & wheel_velocity,
    std::array<double, NUM_CORNERS> & steering_angle,
    std::array<double, NUM_CORNERS> & seek_home_command);
};

}  // namespace rp1_swerve_controller

#endif  // RP1_SWERVE_CONTROLLER__RP1_SWERVE_CONTROLLER_HPP_
