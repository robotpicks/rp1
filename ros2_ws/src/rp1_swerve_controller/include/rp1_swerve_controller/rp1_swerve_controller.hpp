#ifndef RP1_SWERVE_CONTROLLER__RP1_SWERVE_CONTROLLER_HPP_
#define RP1_SWERVE_CONTROLLER__RP1_SWERVE_CONTROLLER_HPP_

#include <array>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"

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

  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    drive_velocity_command_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    steering_position_command_;

  rclcpp::Subscription<TwistStamped>::SharedPtr cmd_vel_subscriber_;
  realtime_tools::RealtimeThreadSafeBox<std::shared_ptr<TwistStamped>> input_cmd_vel_{nullptr};

  // Steering-axis geometry, half-dimensions in metres (base_link frame, X forward/Y left) --
  // defaults match the CAD steering-axis positions in rp1-specs/mechanical_spec.md §3.1
  // (wheelbase 0.8 m -> half 0.4; steering-axis track 1.28 m -> half 0.64), NOT the drive-joint
  // positions (which differ by the ~0.071 m scrub radius, §3.2) -- swerve IK pivots each wheel
  // about its steering axis, so that's the geometry that matters here. Overridable via the
  // half_wheelbase/half_track parameters since these are CAD-unmeasured numbers.
  double half_wheelbase_ = 0.4;
  double half_track_ = 0.64;

  // Per-corner (x, y) in CornerIndex order, computed from half_wheelbase_/half_track_ in
  // on_configure().
  std::array<std::pair<double, double>, NUM_CORNERS> corner_position_{};

  // Last commanded steering angle per corner, radians, UNWRAPPED (not clamped to [-pi, pi]) --
  // continuous joints have no wraparound, and the angle-flip optimization in
  // compute_corner_commands() needs true continuity across cycles to pick the shorter rotation
  // and to avoid the commanded angle jumping by 2*pi at the wrap boundary.
  std::array<double, NUM_CORNERS> last_steering_angle_{};

private:
  // Swerve inverse kinematics: (vx, vy, wz) -> per-corner wheel speed + steering angle.
  // Standard rigid-body-twist decomposition per corner, plus the angle-flip optimization
  // (rotate the wheel <= 90 degrees by allowing negative speed instead of always rotating to the
  // literal computed angle) against last_steering_angle_ so small cmd_vel changes don't spin a
  // wheel 180 degrees when reversing direction would be shorter. Does NOT yet implement the
  // discrete 0/90-locked or 2-wheel operating modes from rp1-specs/requirements.md -- this is
  // continuous free-angle swerve only; mode switching is a separate, not-yet-designed layer on
  // top of this. Mutates last_steering_angle_, so not const.
  void compute_corner_commands(
    const TwistStamped & cmd_vel, std::array<double, NUM_CORNERS> & wheel_velocity,
    std::array<double, NUM_CORNERS> & steering_angle);
};

}  // namespace rp1_swerve_controller

#endif  // RP1_SWERVE_CONTROLLER__RP1_SWERVE_CONTROLLER_HPP_
