#ifndef RP1_SWERVE_CONTROLLER__RP1_SWERVE_CONTROLLER_HPP_
#define RP1_SWERVE_CONTROLLER__RP1_SWERVE_CONTROLLER_HPP_

#include <array>
#include <memory>
#include <string>
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

private:
  // Placeholder for the swerve inverse-kinematics step: today it always commands 0 velocity / 0
  // angle regardless of input, matching a not-yet-implemented controller rather than guessing at
  // math that hasn't been designed yet. See rp1-specs/requirements.md ("Why swerve, and what it
  // needs to do") for the operating modes this needs to support once implemented, and
  // rp1-specs/software_spec.md ("Swerve controller -- not yet started") for the current status.
  void compute_corner_commands(
    const TwistStamped & cmd_vel, std::array<double, NUM_CORNERS> & wheel_velocity,
    std::array<double, NUM_CORNERS> & steering_angle) const;
};

}  // namespace rp1_swerve_controller

#endif  // RP1_SWERVE_CONTROLLER__RP1_SWERVE_CONTROLLER_HPP_
