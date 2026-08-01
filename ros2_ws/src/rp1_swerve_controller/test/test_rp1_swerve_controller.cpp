// Standard ros2_control controller test pattern: construct real CommandInterface/StateInterface
// objects directly (not a mock_components/GenericSystem hardware component), assign them to the
// controller via assign_interfaces(), drive the lifecycle manually, and read/write the same
// objects directly to act as "hardware" -- this exercises compute_corner_commands()/
// compute_body_twist() through the real controller_interface plumbing (parameter declaration,
// interface lookup by name, lifecycle transitions) rather than calling them as free functions.
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "gmock/gmock.h"
#include "controller_interface/controller_interface_params.hpp"
#include "controller_interface/test_utils.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rp1_swerve_controller/rp1_swerve_controller.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace
{
constexpr double kPi = M_PI;

hardware_interface::StateInterface::SharedPtr make_state_interface(
  const std::string & joint_name, const std::string & interface_name)
{
  hardware_interface::InterfaceInfo info;
  info.name = interface_name;
  info.data_type = "double";
  info.size = 1;
  return std::make_shared<hardware_interface::StateInterface>(
    hardware_interface::InterfaceDescription(joint_name, info));
}

hardware_interface::CommandInterface::SharedPtr make_command_interface(
  const std::string & joint_name, const std::string & interface_name)
{
  hardware_interface::InterfaceInfo info;
  info.name = interface_name;
  info.data_type = "double";
  info.size = 1;
  return std::make_shared<hardware_interface::CommandInterface>(
    hardware_interface::InterfaceDescription(joint_name, info));
}
}  // namespace

class RP1SwerveControllerTest : public ::testing::Test
{
public:
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }
  static void TearDownTestCase() { rclcpp::shutdown(); }

  void SetUp() override
  {
    drive_names_ = {"drive_front_left", "drive_front_right", "drive_rear_left",
                    "drive_rear_right"};
    steering_names_ = {"steering_front_left", "steering_front_right", "steering_rear_left",
                       "steering_rear_right"};
    home_sensor_names_ = {"steering_sensors_front_left", "steering_sensors_front_right",
                          "steering_sensors_rear_left", "steering_sensors_rear_right"};

    for (const auto & name : drive_names_)
    {
      drive_velocity_command_.push_back(make_command_interface(name, "velocity"));
      drive_velocity_state_.push_back(make_state_interface(name, "velocity"));
    }
    for (const auto & name : steering_names_)
    {
      steering_position_command_.push_back(make_command_interface(name, "position"));
      steering_position_state_.push_back(make_state_interface(name, "position"));
      steering_seek_home_command_.push_back(make_command_interface(name, "seek_home"));
      steering_brake_command_.push_back(make_command_interface(name, "brake"));
    }
    for (const auto & name : home_sensor_names_)
    {
      // Default to "not confirmed" (0.0/false), matching a fresh hardware bring-up that hasn't
      // homed yet -- tests that exercise LOCKED_0/LOCKED_90/TWO_WHEEL's actual IK math must
      // explicitly mark the relevant corners homed first (see set_home_0deg/set_home_90deg), or
      // the new homing gate in compute_corner_commands() holds every drive command at zero.
      steering_home_0deg_state_.push_back(make_state_interface(name, "home_0deg"));
      steering_home_90deg_state_.push_back(make_state_interface(name, "home_90deg"));
    }
  }

  // Builds and activates a controller with the given extra parameter overrides (on top of
  // drive_joints/steering_joints, always set). Returns nullptr on any lifecycle failure.
  // include_home_sensors=false skips assigning the seek_home/home_0deg/home_90deg interfaces at
  // all -- simulating a hardware component with no physical concept of them (gz_ros2_control's
  // GazeboSimSystem), as opposed to those interfaces merely existing-but-never-set-true (what
  // every other test in this file, and mock hardware, actually look like). Callers passing false
  // must also pass steering_home_sensors_available:false in extra_params, or on_activate() will
  // still try to find interfaces this helper never assigned and the controller will fail to
  // activate -- exactly mirroring the real controller_manager behavior this is testing.
  // include_brake=false does the same for the "brake" command interface, independently --
  // steering_brake_available_ is a separate parameter from steering_home_sensors_available_.
  std::unique_ptr<rp1_swerve_controller::RP1SwerveController> bring_up(
    const std::vector<rclcpp::Parameter> & extra_params = {}, bool include_home_sensors = true,
    bool include_brake = true)
  {
    auto controller = std::make_unique<rp1_swerve_controller::RP1SwerveController>();

    std::vector<rclcpp::Parameter> params{
      rclcpp::Parameter("drive_joints", drive_names_),
      rclcpp::Parameter("steering_joints", steering_names_),
    };
    params.insert(params.end(), extra_params.begin(), extra_params.end());

    controller_interface::ControllerInterfaceParams init_params;
    init_params.controller_name = "rp1_swerve_controller_test";
    init_params.update_rate = 50;
    init_params.controller_manager_update_rate = 50;
    init_params.node_options = rclcpp::NodeOptions().parameter_overrides(params);

    if (controller->init(init_params) != controller_interface::return_type::OK)
    {
      return nullptr;
    }
    if (!controller_interface::configure_succeeds(controller))
    {
      return nullptr;
    }

    std::vector<hardware_interface::LoanedCommandInterface> command_interfaces;
    std::vector<hardware_interface::LoanedStateInterface> state_interfaces;
    for (auto & iface : drive_velocity_command_) command_interfaces.emplace_back(iface);
    for (auto & iface : steering_position_command_) command_interfaces.emplace_back(iface);
    for (auto & iface : drive_velocity_state_) state_interfaces.emplace_back(iface);
    for (auto & iface : steering_position_state_) state_interfaces.emplace_back(iface);
    if (include_home_sensors)
    {
      for (auto & iface : steering_seek_home_command_) command_interfaces.emplace_back(iface);
      for (auto & iface : steering_home_0deg_state_) state_interfaces.emplace_back(iface);
      for (auto & iface : steering_home_90deg_state_) state_interfaces.emplace_back(iface);
    }
    if (include_brake)
    {
      for (auto & iface : steering_brake_command_) command_interfaces.emplace_back(iface);
    }
    controller->assign_interfaces(std::move(command_interfaces), std::move(state_interfaces));

    if (!controller_interface::activate_succeeds(controller))
    {
      return nullptr;
    }
    return controller;
  }

  // Publishes cmd_vel from a throwaway node and spins the controller's own node until the
  // subscription callback has actually run -- update() only ever reads input_cmd_vel_, which is
  // only populated by that callback, so a real spin (not just calling update()) is required.
  void publish_cmd_vel(
    rp1_swerve_controller::RP1SwerveController & controller, double vx, double vy, double wz)
  {
    auto publisher_node = std::make_shared<rclcpp::Node>("test_cmd_vel_publisher");
    auto publisher = publisher_node->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/rp1_swerve_controller_test/cmd_vel", rclcpp::SystemDefaultsQoS());

    geometry_msgs::msg::TwistStamped msg;
    msg.twist.linear.x = vx;
    msg.twist.linear.y = vy;
    msg.twist.angular.z = wz;

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(publisher_node);
    executor.add_node(controller.get_node()->get_node_base_interface());

    // Poll: wait for the subscription to actually see the message rather than a fixed sleep --
    // the publisher/subscriber match is asynchronous even on the same process.
    for (int attempt = 0; attempt < 200; ++attempt)
    {
      publisher->publish(msg);
      executor.spin_some(std::chrono::milliseconds(10));
      if (publisher->get_subscription_count() > 0)
      {
        executor.spin_some(std::chrono::milliseconds(20));
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  void publish_mode(rp1_swerve_controller::RP1SwerveController & controller, uint8_t mode)
  {
    auto publisher_node = std::make_shared<rclcpp::Node>("test_mode_publisher");
    auto publisher = publisher_node->create_publisher<std_msgs::msg::UInt8>(
      "/rp1_swerve_controller_test/mode", rclcpp::SystemDefaultsQoS());

    std_msgs::msg::UInt8 msg;
    msg.data = mode;

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(publisher_node);
    executor.add_node(controller.get_node()->get_node_base_interface());

    for (int attempt = 0; attempt < 200; ++attempt)
    {
      publisher->publish(msg);
      executor.spin_some(std::chrono::milliseconds(10));
      if (publisher->get_subscription_count() > 0)
      {
        executor.spin_some(std::chrono::milliseconds(20));
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  double drive_command(std::size_t corner_index) const
  {
    return drive_velocity_command_[corner_index]->get_optional<double>().value_or(
      std::numeric_limits<double>::quiet_NaN());
  }

  double steering_command(std::size_t corner_index) const
  {
    return steering_position_command_[corner_index]->get_optional<double>().value_or(
      std::numeric_limits<double>::quiet_NaN());
  }

  double seek_home_command(std::size_t corner_index) const
  {
    return steering_seek_home_command_[corner_index]->get_optional<double>().value_or(
      std::numeric_limits<double>::quiet_NaN());
  }

  double brake_command(std::size_t corner_index) const
  {
    return steering_brake_command_[corner_index]->get_optional<double>().value_or(
      std::numeric_limits<double>::quiet_NaN());
  }

  // Sets every corner's home_0deg (or home_90deg) gpio state, simulating a hardware bring-up
  // that has already homed -- tests exercising LOCKED_0/LOCKED_90/TWO_WHEEL's actual IK math (as
  // opposed to the homing gate itself) call this first so the new gate in
  // compute_corner_commands() doesn't hold every drive command at zero.
  void set_all_home_0deg(bool confirmed)
  {
    for (auto & iface : steering_home_0deg_state_)
    {
      ASSERT_TRUE(iface->set_value<double>(confirmed ? 1.0 : 0.0));
    }
  }
  void set_all_home_90deg(bool confirmed)
  {
    for (auto & iface : steering_home_90deg_state_)
    {
      ASSERT_TRUE(iface->set_value<double>(confirmed ? 1.0 : 0.0));
    }
  }

  // Drives every drive/steering STATE interface to a uniform (velocity, angle) pair -- i.e. all
  // 4 corners "reporting" the same wheel speed pointed the same direction, so
  // compute_body_twist() recovers a pure vx of `velocity_rad_s * wheel_radius` (default
  // wheel_radius is 0.2154) with vy=wz=0. Used to simulate "the chassis is actually moving" (or
  // not) for the mode-switch-safety gate, independent of whatever's been commanded -- this test
  // harness has no hardware component looping commands back to state, so state has to be driven
  // by hand.
  void set_uniform_drive_state(double velocity_rad_s, double angle_rad = 0.0)
  {
    for (auto & iface : drive_velocity_state_)
    {
      ASSERT_TRUE(iface->set_value<double>(velocity_rad_s));
    }
    for (auto & iface : steering_position_state_)
    {
      ASSERT_TRUE(iface->set_value<double>(angle_rad));
    }
  }

  std::vector<std::string> drive_names_;
  std::vector<std::string> steering_names_;
  std::vector<std::string> home_sensor_names_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> drive_velocity_command_;
  std::vector<hardware_interface::StateInterface::SharedPtr> drive_velocity_state_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> steering_position_command_;
  std::vector<hardware_interface::StateInterface::SharedPtr> steering_position_state_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> steering_seek_home_command_;
  std::vector<hardware_interface::StateInterface::SharedPtr> steering_home_0deg_state_;
  std::vector<hardware_interface::StateInterface::SharedPtr> steering_home_90deg_state_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> steering_brake_command_;
};

// CornerIndex order used throughout: front_left, front_right, rear_left, rear_right.

TEST_F(RP1SwerveControllerTest, StraightDriveAllWheelsZeroAngleFullSpeed)
{
  auto controller = bring_up();
  ASSERT_NE(controller, nullptr);

  publish_cmd_vel(*controller, 1.0, 0.0, 0.0);
  auto time = controller->get_node()->now();
  rclcpp::Duration period(std::chrono::milliseconds(20));
  ASSERT_EQ(controller->update(time, period), controller_interface::return_type::OK);

  for (std::size_t i = 0; i < 4; ++i)
  {
    EXPECT_NEAR(steering_command(i), 0.0, 1e-9) << "corner " << i;
    EXPECT_NEAR(drive_command(i), 4.642525533890436, 1e-9) << "corner " << i;
  }
}

TEST_F(RP1SwerveControllerTest, PureCrabAllWheelsNinetyDegrees)
{
  auto controller = bring_up();
  ASSERT_NE(controller, nullptr);

  publish_cmd_vel(*controller, 0.0, 0.5, 0.0);
  auto time = controller->get_node()->now();
  rclcpp::Duration period(std::chrono::milliseconds(20));
  ASSERT_EQ(controller->update(time, period), controller_interface::return_type::OK);

  for (std::size_t i = 0; i < 4; ++i)
  {
    EXPECT_NEAR(steering_command(i), kPi / 2.0, 1e-9) << "corner " << i;
    EXPECT_NEAR(drive_command(i), 2.321262766945218, 1e-9) << "corner " << i;
  }
}

TEST_F(RP1SwerveControllerTest, TurnInPlaceAngleFlipOptimizationEngages)
{
  auto controller = bring_up();
  ASSERT_NE(controller, nullptr);

  // Prime last_steering_angle_ at 0 for every corner (matches a fresh controller's default), so
  // the flip decision below is against a known starting point.
  publish_cmd_vel(*controller, 1.0, 0.0, 0.0);
  rclcpp::Duration period(std::chrono::milliseconds(20));
  controller->update(controller->get_node()->now(), period);

  publish_cmd_vel(*controller, 0.0, 0.0, 1.0);
  controller->update(controller->get_node()->now(), period);

  // FRONT_LEFT (x=0.4, y=0.64): raw angle atan2(0.4, -0.64) ~= 2.583 rad, further than +/- pi/2
  // from 0 -- expect the flip: angle ~= -0.5586 rad, speed negated to ~= -0.7547.
  EXPECT_NEAR(steering_command(0), -0.5585993153435624, 1e-9);
  EXPECT_NEAR(drive_command(0), -3.503799863345071, 1e-9);
  // FRONT_RIGHT (x=0.4, y=-0.64): raw angle atan2(0.4, 0.64) ~= 0.5586 rad, within pi/2 -- no flip.
  EXPECT_NEAR(steering_command(1), 0.5585993153435624, 1e-9);
  EXPECT_NEAR(drive_command(1), 3.503799863345071, 1e-9);
}

TEST_F(RP1SwerveControllerTest, Locked0IgnoresLateralAndSkidSteers)
{
  auto controller = bring_up();
  ASSERT_NE(controller, nullptr);

  set_all_home_0deg(true);  // already homed -- isolates this test to the locked-projection math
  publish_mode(*controller, 1);  // LOCKED_0
  publish_cmd_vel(*controller, 0.5, 0.0, 0.3);
  rclcpp::Duration period(std::chrono::milliseconds(20));
  ASSERT_EQ(
    controller->update(controller->get_node()->now(), period),
    controller_interface::return_type::OK);

  for (std::size_t i = 0; i < 4; ++i)
  {
    EXPECT_NEAR(steering_command(i), 0.0, 1e-9) << "corner " << i;
  }
  // Left corners (front_left=0, rear_left=2, y=+0.64) slower; right corners faster -- standard
  // skid-steer differential, vx -/+ wz*half_track.
  EXPECT_NEAR(drive_command(0), 1.4298978644382543, 1e-9);   // front_left
  EXPECT_NEAR(drive_command(1), 3.2126276694521816, 1e-9);   // front_right
  EXPECT_NEAR(drive_command(2), 1.4298978644382543, 1e-9);   // rear_left
  EXPECT_NEAR(drive_command(3), 3.2126276694521816, 1e-9);   // rear_right
}

TEST_F(RP1SwerveControllerTest, Locked90IgnoresForwardAndCrabsWithFrontRearSplit)
{
  auto controller = bring_up();
  ASSERT_NE(controller, nullptr);

  set_all_home_90deg(true);  // already homed -- isolates this test to the locked-projection math
  publish_mode(*controller, 2);  // LOCKED_90
  publish_cmd_vel(*controller, 0.0, 0.5, 0.3);
  rclcpp::Duration period(std::chrono::milliseconds(20));
  ASSERT_EQ(
    controller->update(controller->get_node()->now(), period),
    controller_interface::return_type::OK);

  for (std::size_t i = 0; i < 4; ++i)
  {
    EXPECT_NEAR(steering_command(i), kPi / 2.0, 1e-9) << "corner " << i;
  }
  EXPECT_NEAR(drive_command(0), 2.8783658310120703, 1e-9);  // front_left
  EXPECT_NEAR(drive_command(1), 2.8783658310120703, 1e-9);  // front_right
  EXPECT_NEAR(drive_command(2), 1.7641597028783658, 1e-9);  // rear_left
  EXPECT_NEAR(drive_command(3), 1.7641597028783658, 1e-9);  // rear_right
}

TEST_F(RP1SwerveControllerTest, TwoWheelDefaultPairIsFrontFreeRearLocked)
{
  auto controller = bring_up();
  ASSERT_NE(controller, nullptr);

  set_all_home_0deg(true);  // locked pair's reference already confirmed
  publish_mode(*controller, 3);  // TWO_WHEEL
  publish_cmd_vel(*controller, 0.5, 0.2, 0.1);
  rclcpp::Duration period(std::chrono::milliseconds(20));
  ASSERT_EQ(
    controller->update(controller->get_node()->now(), period),
    controller_interface::return_type::OK);

  // Front corners free-steer (nonzero angle).
  EXPECT_NEAR(steering_command(0), 0.5031953235893651, 1e-9);
  EXPECT_NEAR(steering_command(1), 0.4023210978604406, 1e-9);
  // Rear corners locked at 0, skid-differential speed.
  EXPECT_NEAR(steering_command(2), 0.0, 1e-9);
  EXPECT_NEAR(steering_command(3), 0.0, 1e-9);
  EXPECT_NEAR(drive_command(2), 2.02414113277623, 1e-9);
  EXPECT_NEAR(drive_command(3), 2.6183844011142057, 1e-9);
}

TEST_F(RP1SwerveControllerTest, TwoWheelCustomPairMovesWhichCornersAreLocked)
{
  auto controller = bring_up(
    {rclcpp::Parameter(
      "two_wheel_steered_corners", std::vector<std::string>{"front_left", "rear_left"})});
  ASSERT_NE(controller, nullptr);

  publish_mode(*controller, 3);  // TWO_WHEEL
  publish_cmd_vel(*controller, 0.5, 0.2, 0.1);
  rclcpp::Duration period(std::chrono::milliseconds(20));
  ASSERT_EQ(
    controller->update(controller->get_node()->now(), period),
    controller_interface::return_type::OK);

  // Now front_left/rear_left free-steer (nonzero), front_right/rear_right locked.
  EXPECT_NE(steering_command(0), 0.0);   // front_left, free
  EXPECT_NEAR(steering_command(1), 0.0, 1e-9);  // front_right, now locked
  EXPECT_NE(steering_command(2), 0.0);   // rear_left, free
  EXPECT_NEAR(steering_command(3), 0.0, 1e-9);  // rear_right, still locked
}

TEST_F(RP1SwerveControllerTest, Locked0BlocksDriveAndRequestsSeekHomeUntilConfirmed)
{
  auto controller = bring_up();
  ASSERT_NE(controller, nullptr);

  // Fresh bring-up: home_0deg defaults to false/unconfirmed (see SetUp()'s comment). Switching
  // into LOCKED_0 must NOT immediately drive on an unverified locked angle (rp1-specs/
  // requirements.md's "Transitioning between the two locked configurations..." note) -- every
  // drive wheel should be held at zero and a fresh seek_home(0.0) request issued instead.
  publish_mode(*controller, 1);  // LOCKED_0
  publish_cmd_vel(*controller, 0.5, 0.0, 0.3);
  rclcpp::Duration period(std::chrono::milliseconds(20));
  ASSERT_EQ(
    controller->update(controller->get_node()->now(), period),
    controller_interface::return_type::OK);

  for (std::size_t i = 0; i < 4; ++i)
  {
    EXPECT_NEAR(drive_command(i), 0.0, 1e-9) << "corner " << i << " should be held while unhomed";
    EXPECT_NEAR(seek_home_command(i), 0.0, 1e-9)
      << "corner " << i << " should request a seek to the 0deg reference";
  }

  // Now confirm the physical reference on every corner (as if the firmware's homing_tick() just
  // completed) and re-run: driving should resume exactly as if it had been homed all along, and
  // no further seek_home request should be pending (NaN = no active request).
  set_all_home_0deg(true);
  ASSERT_EQ(
    controller->update(controller->get_node()->now(), period),
    controller_interface::return_type::OK);

  EXPECT_NEAR(drive_command(0), 1.4298978644382543, 1e-9);
  EXPECT_NEAR(drive_command(1), 3.2126276694521816, 1e-9);
  EXPECT_NEAR(drive_command(2), 1.4298978644382543, 1e-9);
  EXPECT_NEAR(drive_command(3), 3.2126276694521816, 1e-9);
  for (std::size_t i = 0; i < 4; ++i)
  {
    EXPECT_TRUE(std::isnan(seek_home_command(i))) << "corner " << i;
  }
}

TEST_F(RP1SwerveControllerTest, ModeSwitchDeferredWhileChassisIsMovingThenTakesEffectOnceStopped)
{
  auto controller = bring_up();
  ASSERT_NE(controller, nullptr);

  // Already homed at 0deg -- isolates this test to the mode-switch-safety gate, not the homing
  // gate (both apply independently; see active_mode_'s header comment).
  set_all_home_0deg(true);
  // Simulate the chassis actually moving: 5 rad/s * wheel_radius (0.2154) ~= 1.08 m/s, well
  // above mode_switch_stopped_tolerance_'s 0.02 default.
  set_uniform_drive_state(5.0, 0.0);

  publish_mode(*controller, 1);  // LOCKED_0
  publish_cmd_vel(*controller, 0.5, 0.0, 0.3);
  rclcpp::Duration period(std::chrono::milliseconds(20));
  ASSERT_EQ(
    controller->update(controller->get_node()->now(), period),
    controller_interface::return_type::OK);

  // Still moving -> the switch to LOCKED_0 must not have taken effect yet: this is exactly the
  // same (vx=0.5, vy=0, wz=0.3) cmd_vel as TurnInPlaceAngleFlipOptimizationEngages-style inputs,
  // so FULL_SWERVE's IK should still be running, meaning steering angles are NOT pinned to 0.
  for (std::size_t i = 0; i < 4; ++i)
  {
    EXPECT_NE(steering_command(i), 0.0) << "corner " << i << " should still be free (FULL_SWERVE)";
  }

  // Now the chassis reports actually stopped -- the deferred switch should take effect on the
  // very next cycle, producing the exact same numbers Locked0IgnoresLateralAndSkidSteers checks.
  set_uniform_drive_state(0.0, 0.0);
  ASSERT_EQ(
    controller->update(controller->get_node()->now(), period),
    controller_interface::return_type::OK);

  for (std::size_t i = 0; i < 4; ++i)
  {
    EXPECT_NEAR(steering_command(i), 0.0, 1e-9) << "corner " << i;
  }
  EXPECT_NEAR(drive_command(0), 1.4298978644382543, 1e-9);
  EXPECT_NEAR(drive_command(1), 3.2126276694521816, 1e-9);
  EXPECT_NEAR(drive_command(2), 1.4298978644382543, 1e-9);
  EXPECT_NEAR(drive_command(3), 3.2126276694521816, 1e-9);
}

TEST_F(RP1SwerveControllerTest, ModeSwitchRampsCommandedTwistTowardZeroRegardlessOfContinuedCmdVel)
{
  auto controller = bring_up();
  ASSERT_NE(controller, nullptr);

  set_all_home_0deg(true);
  // Chassis reads as permanently "still moving" in this harness (nothing loops state feedback
  // back to whatever's commanded), so the mode switch stays pending for as long as this test
  // keeps calling update() -- exactly what's needed to observe the ramp settle at zero rather
  // than just its first step.
  set_uniform_drive_state(5.0, 0.0);

  publish_mode(*controller, 1);  // LOCKED_0
  // Pure +x: angle stays 0 under FULL_SWERVE's IK (vy=wz=0), so only drive_command needs
  // checking here -- steering_command under a pending switch is exercised by the sibling
  // ModeSwitchDeferredWhileChassisIsMovingThenTakesEffectOnceStopped test above. ~/cmd_vel keeps
  // holding this same nonzero value for the rest of the test (nothing re-publishes it), so a
  // decreasing drive_command below proves the ramp decelerates its own frozen state rather than
  // tracking whatever ~/cmd_vel says every cycle.
  publish_cmd_vel(*controller, 1.0, 0.0, 0.0);
  rclcpp::Duration period(std::chrono::milliseconds(20));

  // mode_switch_linear_deceleration_ defaults to 0.5 m/s^2, so each 20ms cycle should knock
  // 0.01 m/s (0.01 / 0.2154 rad/s on the wheel) off the ramp -- verify a strictly decreasing
  // sequence over a few cycles, not just the eventual zero, so a bug that clamped straight to
  // zero (skipping the ramp entirely) would still be caught.
  double previous = std::numeric_limits<double>::infinity();
  for (int cycle = 0; cycle < 5; ++cycle)
  {
    ASSERT_EQ(
      controller->update(controller->get_node()->now(), period),
      controller_interface::return_type::OK);
    const double current = drive_command(0);
    EXPECT_LT(current, previous) << "cycle " << cycle;
    EXPECT_GT(current, 0.0) << "cycle " << cycle;
    previous = current;
  }

  // Run enough further cycles for the ramp to fully bottom out at zero and STAY there (not
  // oscillate past zero) even though the mode switch never gets to commit in this harness (state
  // is pinned artificially nonzero above, so is_stopped is never true).
  for (int cycle = 0; cycle < 200; ++cycle)
  {
    ASSERT_EQ(
      controller->update(controller->get_node()->now(), period),
      controller_interface::return_type::OK);
  }
  EXPECT_NEAR(drive_command(0), 0.0, 1e-9);
}

TEST_F(RP1SwerveControllerTest, ModeSwitchTakesEffectImmediatelyWhenAlreadyStopped)
{
  auto controller = bring_up();
  ASSERT_NE(controller, nullptr);

  // Default (never-driven) state reads back as 0 -- see set_uniform_drive_state()'s absence
  // here: a controller that's never received any state feedback is treated as stopped, not
  // blocked indefinitely (this is also why every other test in this file, which never calls
  // set_uniform_drive_state(), has always seen mode switches take effect on the very first
  // update() call).
  set_all_home_0deg(true);
  publish_mode(*controller, 1);  // LOCKED_0
  publish_cmd_vel(*controller, 0.5, 0.0, 0.3);
  rclcpp::Duration period(std::chrono::milliseconds(20));
  ASSERT_EQ(
    controller->update(controller->get_node()->now(), period),
    controller_interface::return_type::OK);

  for (std::size_t i = 0; i < 4; ++i)
  {
    EXPECT_NEAR(steering_command(i), 0.0, 1e-9) << "corner " << i;
  }
}

TEST_F(RP1SwerveControllerTest, StaleCmdVelZerosDriveButHoldsSteeringAngle)
{
  auto controller = bring_up();
  ASSERT_NE(controller, nullptr);

  // Pure crab: all four corners at +90 degrees, wheels turning. publish_cmd_vel() leaves
  // header.stamp zero, so the subscription callback stamps it at reception -- `time` here is
  // taken right after, making the command fresh for the first update().
  publish_cmd_vel(*controller, 0.0, 0.5, 0.0);
  auto time = controller->get_node()->now();
  rclcpp::Duration period(std::chrono::milliseconds(20));
  ASSERT_EQ(controller->update(time, period), controller_interface::return_type::OK);
  for (std::size_t i = 0; i < 4; ++i)
  {
    EXPECT_NEAR(drive_command(i), 2.321262766945218, 1e-9) << "corner " << i;
  }

  // Publisher goes silent past the (default 0.5 s) timeout: drive must zero, steering must
  // HOLD the crab angle rather than snap back to 0 -- a wheel re-steering to straight while
  // the chassis rolls out sideways would scrub, which is the failure mode this ordering
  // guards. 2 s leaves generous slack over the callback-to-now() stamping gap.
  ASSERT_EQ(
    controller->update(time + rclcpp::Duration::from_seconds(2.0), period),
    controller_interface::return_type::OK);
  for (std::size_t i = 0; i < 4; ++i)
  {
    EXPECT_NEAR(drive_command(i), 0.0, 1e-9) << "corner " << i;
    EXPECT_NEAR(steering_command(i), kPi / 2.0, 1e-9) << "corner " << i;
  }

  // A fresh command revives driving -- the watchdog gates on message age, not on any latched
  // "timed out" state.
  publish_cmd_vel(*controller, 0.0, 0.5, 0.0);
  ASSERT_EQ(
    controller->update(controller->get_node()->now(), period),
    controller_interface::return_type::OK);
  for (std::size_t i = 0; i < 4; ++i)
  {
    EXPECT_NEAR(drive_command(i), 2.321262766945218, 1e-9) << "corner " << i;
  }
}

TEST_F(RP1SwerveControllerTest, CmdVelWithinTimeoutKeepsDriving)
{
  // Generous timeout so the deliberately-aged update() below is unambiguously fresh no matter
  // how slowly the publish helper's spin loop ran.
  auto controller = bring_up({rclcpp::Parameter("cmd_vel_timeout", 10.0)});
  ASSERT_NE(controller, nullptr);

  publish_cmd_vel(*controller, 1.0, 0.0, 0.0);
  auto time = controller->get_node()->now();
  rclcpp::Duration period(std::chrono::milliseconds(20));
  ASSERT_EQ(
    controller->update(time + rclcpp::Duration::from_seconds(5.0), period),
    controller_interface::return_type::OK);
  for (std::size_t i = 0; i < 4; ++i)
  {
    EXPECT_NEAR(drive_command(i), 4.642525533890436, 1e-9) << "corner " << i;
  }
}

TEST_F(RP1SwerveControllerTest, NonPositiveCmdVelTimeoutFailsInit)
{
  // Zero would mark every command stale on arrival -- a config error, not a "disable" switch.
  auto controller = bring_up({rclcpp::Parameter("cmd_vel_timeout", 0.0)});
  EXPECT_EQ(controller, nullptr);
}

TEST_F(RP1SwerveControllerTest, Locked90BlocksDriveUntilConfirmedAtNinetyDegrees)
{
  auto controller = bring_up();
  ASSERT_NE(controller, nullptr);

  publish_mode(*controller, 2);  // LOCKED_90
  publish_cmd_vel(*controller, 0.0, 0.5, 0.3);
  rclcpp::Duration period(std::chrono::milliseconds(20));
  ASSERT_EQ(
    controller->update(controller->get_node()->now(), period),
    controller_interface::return_type::OK);

  for (std::size_t i = 0; i < 4; ++i)
  {
    EXPECT_NEAR(drive_command(i), 0.0, 1e-9) << "corner " << i << " should be held while unhomed";
    EXPECT_NEAR(seek_home_command(i), 1.0, 1e-9)
      << "corner " << i << " should request a seek to the 90deg reference";
  }

  set_all_home_90deg(true);
  ASSERT_EQ(
    controller->update(controller->get_node()->now(), period),
    controller_interface::return_type::OK);

  EXPECT_NEAR(drive_command(0), 2.8783658310120703, 1e-9);
  EXPECT_NEAR(drive_command(1), 2.8783658310120703, 1e-9);
  EXPECT_NEAR(drive_command(2), 1.7641597028783658, 1e-9);
  EXPECT_NEAR(drive_command(3), 1.7641597028783658, 1e-9);
}

TEST_F(RP1SwerveControllerTest, Locked0PartiallyHomedStillBlocksAllDrive)
{
  auto controller = bring_up();
  ASSERT_NE(controller, nullptr);

  // Only 3 of 4 corners confirmed -- the whole chassis must still stay put, not just the one
  // unconfirmed corner (mixing a confirmed and unconfirmed locked wheel would scrub/drag if the
  // drive wheels moved).
  ASSERT_TRUE(steering_home_0deg_state_[0]->set_value<double>(1.0));
  ASSERT_TRUE(steering_home_0deg_state_[1]->set_value<double>(1.0));
  ASSERT_TRUE(steering_home_0deg_state_[2]->set_value<double>(1.0));
  ASSERT_TRUE(steering_home_0deg_state_[3]->set_value<double>(0.0));

  publish_mode(*controller, 1);  // LOCKED_0
  publish_cmd_vel(*controller, 0.5, 0.0, 0.3);
  rclcpp::Duration period(std::chrono::milliseconds(20));
  ASSERT_EQ(
    controller->update(controller->get_node()->now(), period),
    controller_interface::return_type::OK);

  for (std::size_t i = 0; i < 4; ++i)
  {
    EXPECT_NEAR(drive_command(i), 0.0, 1e-9) << "corner " << i;
  }
  EXPECT_TRUE(std::isnan(seek_home_command(0)));  // already confirmed, no active request
  EXPECT_NEAR(seek_home_command(3), 0.0, 1e-9);   // still seeking
}

TEST_F(RP1SwerveControllerTest, TwoWheelOnlyLockedPairGatesDrive)
{
  auto controller = bring_up();
  ASSERT_NE(controller, nullptr);

  // Default two_wheel_steered_corners is the front pair -- rear corners are the locked ones and
  // need home_0deg; front corners free-steer and never gate on anything.
  publish_mode(*controller, 3);  // TWO_WHEEL
  publish_cmd_vel(*controller, 0.5, 0.2, 0.1);
  rclcpp::Duration period(std::chrono::milliseconds(20));
  ASSERT_EQ(
    controller->update(controller->get_node()->now(), period),
    controller_interface::return_type::OK);

  // Unconfirmed rear (locked) pair holds the whole chassis, including the free front corners.
  for (std::size_t i = 0; i < 4; ++i)
  {
    EXPECT_NEAR(drive_command(i), 0.0, 1e-9) << "corner " << i;
  }
  EXPECT_TRUE(std::isnan(seek_home_command(0)));  // front_left, free -- never requests a seek
  EXPECT_TRUE(std::isnan(seek_home_command(1)));  // front_right, free
  EXPECT_NEAR(seek_home_command(2), 0.0, 1e-9);   // rear_left, locked, unconfirmed
  EXPECT_NEAR(seek_home_command(3), 0.0, 1e-9);   // rear_right, locked, unconfirmed

  set_all_home_0deg(true);
  ASSERT_EQ(
    controller->update(controller->get_node()->now(), period),
    controller_interface::return_type::OK);
  EXPECT_NEAR(drive_command(2), 2.02414113277623, 1e-9);
  EXPECT_NEAR(drive_command(3), 2.6183844011142057, 1e-9);
}

TEST_F(RP1SwerveControllerTest, FullSwerveNeverGatesOnHoming)
{
  auto controller = bring_up();
  ASSERT_NE(controller, nullptr);

  // Default mode is FULL_SWERVE, and home_0deg/home_90deg default to unconfirmed -- full swerve
  // has no fixed reference angle to verify, so it must drive immediately regardless.
  publish_cmd_vel(*controller, 1.0, 0.0, 0.0);
  rclcpp::Duration period(std::chrono::milliseconds(20));
  ASSERT_EQ(
    controller->update(controller->get_node()->now(), period),
    controller_interface::return_type::OK);

  for (std::size_t i = 0; i < 4; ++i)
  {
    EXPECT_NEAR(drive_command(i), 4.642525533890436, 1e-9) << "corner " << i;
    EXPECT_TRUE(std::isnan(seek_home_command(i))) << "corner " << i;
    // Free-steering corners are actively tracking a moving target -- the brake must stay
    // released, never engage on a corner that needs to be free to turn.
    EXPECT_NEAR(brake_command(i), 0.0, 1e-9) << "corner " << i;
  }
}

TEST_F(RP1SwerveControllerTest, Locked0EngagesBrakeOnceConfirmedReleasesWhilePending)
{
  auto controller = bring_up();
  ASSERT_NE(controller, nullptr);

  publish_mode(*controller, 1);  // LOCKED_0
  publish_cmd_vel(*controller, 0.5, 0.0, 0.3);
  rclcpp::Duration period(std::chrono::milliseconds(20));

  // Unconfirmed (fresh bring-up, home_0deg defaults false): still seeking, so the brake must
  // stay released -- the firmware's homing_tick() needs the shaft free to turn, and engaging
  // here would fight that seek.
  ASSERT_EQ(
    controller->update(controller->get_node()->now(), period),
    controller_interface::return_type::OK);
  for (std::size_t i = 0; i < 4; ++i)
  {
    EXPECT_NEAR(brake_command(i), 0.0, 1e-9) << "corner " << i << " should be released while unhomed";
  }

  // Confirmed: the corner is now trustworthy enough to stop actively driving it (the drive-zero
  // gate lifts, per Locked0BlocksDriveAndRequestsSeekHomeUntilConfirmed) -- the brake should
  // engage at the same moment, to hold the angle without continuous motor current.
  set_all_home_0deg(true);
  ASSERT_EQ(
    controller->update(controller->get_node()->now(), period),
    controller_interface::return_type::OK);
  for (std::size_t i = 0; i < 4; ++i)
  {
    EXPECT_NEAR(brake_command(i), 1.0, 1e-9) << "corner " << i << " should engage once confirmed";
  }
}

TEST_F(RP1SwerveControllerTest, TwoWheelEngagesBrakeOnlyOnTheLockedPair)
{
  auto controller = bring_up();
  ASSERT_NE(controller, nullptr);

  set_all_home_0deg(true);  // locked pair's reference already confirmed
  publish_mode(*controller, 3);  // TWO_WHEEL
  publish_cmd_vel(*controller, 0.5, 0.2, 0.1);
  rclcpp::Duration period(std::chrono::milliseconds(20));
  ASSERT_EQ(
    controller->update(controller->get_node()->now(), period),
    controller_interface::return_type::OK);

  // Default two_wheel_steered_corners is the front pair -- front corners free-steer (brake
  // released), rear corners are locked-and-confirmed (brake engaged).
  EXPECT_NEAR(brake_command(0), 0.0, 1e-9);  // front_left, free
  EXPECT_NEAR(brake_command(1), 0.0, 1e-9);  // front_right, free
  EXPECT_NEAR(brake_command(2), 1.0, 1e-9);  // rear_left, locked + confirmed
  EXPECT_NEAR(brake_command(3), 1.0, 1e-9);  // rear_right, locked + confirmed
}

TEST_F(RP1SwerveControllerTest, ActivatesAndDrivesFullSwerveWithoutAnyHomeSensorInterfaces)
{
  // Mirrors gz_ros2_control's GazeboSimSystem: no seek_home/home_0deg/home_90deg/brake
  // interfaces exist anywhere in the system at all (not just unconfirmed, as every other test in
  // this file has them). steering_home_sensors_available/steering_brake_available:false must be
  // set, or on_activate() would still try to find interfaces this bring_up() call never assigned
  // and fail to activate -- the same failure these parameters exist to avoid (see the
  // controller's README).
  auto controller = bring_up(
    {rclcpp::Parameter("steering_home_sensors_available", false),
     rclcpp::Parameter("steering_brake_available", false)},
    /*include_home_sensors=*/false, /*include_brake=*/false);
  ASSERT_NE(controller, nullptr);

  publish_cmd_vel(*controller, 1.0, 0.0, 0.0);
  rclcpp::Duration period(std::chrono::milliseconds(20));
  ASSERT_EQ(
    controller->update(controller->get_node()->now(), period),
    controller_interface::return_type::OK);

  // Same numbers as StraightDriveAllWheelsZeroAngleFullSpeed -- FULL_SWERVE never needed home
  // sensors in the first place.
  for (std::size_t i = 0; i < 4; ++i)
  {
    EXPECT_NEAR(steering_command(i), 0.0, 1e-9) << "corner " << i;
    EXPECT_NEAR(drive_command(i), 4.642525533890436, 1e-9) << "corner " << i;
  }
}

TEST_F(RP1SwerveControllerTest, Locked0StaysGatedForeverWithoutAnyHomeSensorInterfaces)
{
  auto controller = bring_up(
    {rclcpp::Parameter("steering_home_sensors_available", false),
     rclcpp::Parameter("steering_brake_available", false)},
    /*include_home_sensors=*/false, /*include_brake=*/false);
  ASSERT_NE(controller, nullptr);

  publish_mode(*controller, 1);  // LOCKED_0
  publish_cmd_vel(*controller, 0.5, 0.0, 0.3);
  rclcpp::Duration period(std::chrono::milliseconds(20));

  // Run several cycles -- with no home sensor interfaces at all, there is nothing that could
  // ever confirm the reference, so this must stay gated indefinitely, not just on the first
  // cycle (matching what was verified live: rp1_swerve_mock.launch.py and
  // rp1_swerve_gazebo.launch.py both hold LOCKED_0 at zero forever, for the same reason).
  for (int cycle = 0; cycle < 5; ++cycle)
  {
    ASSERT_EQ(
      controller->update(controller->get_node()->now(), period),
      controller_interface::return_type::OK);
    for (std::size_t i = 0; i < 4; ++i)
    {
      EXPECT_NEAR(drive_command(i), 0.0, 1e-9) << "cycle " << cycle << " corner " << i;
    }
  }
}

TEST_F(RP1SwerveControllerTest, UnrecognizedModeFallsBackToFullSwerve)
{
  auto controller = bring_up();
  ASSERT_NE(controller, nullptr);

  publish_mode(*controller, 99);  // not a valid SwerveMode
  publish_cmd_vel(*controller, 1.0, 0.0, 0.0);
  rclcpp::Duration period(std::chrono::milliseconds(20));
  ASSERT_EQ(
    controller->update(controller->get_node()->now(), period),
    controller_interface::return_type::OK);

  for (std::size_t i = 0; i < 4; ++i)
  {
    EXPECT_NEAR(steering_command(i), 0.0, 1e-9) << "corner " << i;
    EXPECT_NEAR(drive_command(i), 4.642525533890436, 1e-9) << "corner " << i;
  }
}

TEST_F(RP1SwerveControllerTest, OdometryIntegratesStraightLineDistance)
{
  auto controller = bring_up();
  ASSERT_NE(controller, nullptr);

  // Drive the state interfaces directly (playing "hardware": on real/mock hardware these would
  // reflect actual reported wheel speed/position) to a forward-rolling, unsteered configuration.
  // wheel_radius default is 0.2154, so an angular velocity state of 1.0/0.2154 rad/s corresponds
  // to 1.0 m/s linear -- compute_body_twist() should recover vx=1.0, vy=0, wz=0 from this.
  for (auto & iface : steering_position_state_)
  {
    ASSERT_TRUE(iface->set_value<double>(0.0));
  }
  for (auto & iface : drive_velocity_state_)
  {
    ASSERT_TRUE(iface->set_value<double>(1.0 / 0.2154));
  }

  auto subscriber_node = std::make_shared<rclcpp::Node>("test_odom_subscriber");
  nav_msgs::msg::Odometry latest_odom;
  bool got_odom = false;
  auto subscription = subscriber_node->create_subscription<nav_msgs::msg::Odometry>(
    "/rp1_swerve_controller_test/odom", rclcpp::SystemDefaultsQoS(),
    [&latest_odom, &got_odom](const nav_msgs::msg::Odometry & msg)
    {
      latest_odom = msg;
      got_odom = true;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(subscriber_node);
  executor.add_node(controller->get_node()->get_node_base_interface());

  rclcpp::Duration period(std::chrono::milliseconds(20));
  for (int i = 0; i < 50; ++i)
  {
    controller->update(controller->get_node()->now(), period);
    executor.spin_some(std::chrono::milliseconds(5));
  }
  // A few extra spins in case the realtime publisher's background thread hasn't caught up.
  for (int attempt = 0; attempt < 50 && !got_odom; ++attempt)
  {
    executor.spin_some(std::chrono::milliseconds(10));
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  ASSERT_TRUE(got_odom) << "never received an ~/odom message";
  // 50 cycles * 0.02s * 1.0 m/s = 1.0 m -- some tolerance since try_publish() can drop a cycle
  // under contention and the exact cycle count reaching the subscriber isn't guaranteed.
  EXPECT_NEAR(latest_odom.pose.pose.position.x, 1.0, 0.05);
  EXPECT_NEAR(latest_odom.pose.pose.position.y, 0.0, 1e-6);
  EXPECT_NEAR(latest_odom.twist.twist.linear.x, 1.0, 1e-6);
}
