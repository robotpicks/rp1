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

    for (const auto & name : drive_names_)
    {
      drive_velocity_command_.push_back(make_command_interface(name, "velocity"));
      drive_velocity_state_.push_back(make_state_interface(name, "velocity"));
    }
    for (const auto & name : steering_names_)
    {
      steering_position_command_.push_back(make_command_interface(name, "position"));
      steering_position_state_.push_back(make_state_interface(name, "position"));
    }
  }

  // Builds and activates a controller with the given extra parameter overrides (on top of
  // drive_joints/steering_joints, always set). Returns nullptr on any lifecycle failure.
  std::unique_ptr<rp1_swerve_controller::RP1SwerveController> bring_up(
    const std::vector<rclcpp::Parameter> & extra_params = {})
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

  std::vector<std::string> drive_names_;
  std::vector<std::string> steering_names_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> drive_velocity_command_;
  std::vector<hardware_interface::StateInterface::SharedPtr> drive_velocity_state_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> steering_position_command_;
  std::vector<hardware_interface::StateInterface::SharedPtr> steering_position_state_;
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
    EXPECT_NEAR(drive_command(i), 1.0, 1e-9) << "corner " << i;
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
    EXPECT_NEAR(drive_command(i), 0.5, 1e-9) << "corner " << i;
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
  EXPECT_NEAR(drive_command(0), -0.7547184905645283, 1e-9);
  // FRONT_RIGHT (x=0.4, y=-0.64): raw angle atan2(0.4, 0.64) ~= 0.5586 rad, within pi/2 -- no flip.
  EXPECT_NEAR(steering_command(1), 0.5585993153435624, 1e-9);
  EXPECT_NEAR(drive_command(1), 0.7547184905645283, 1e-9);
}

TEST_F(RP1SwerveControllerTest, Locked0IgnoresLateralAndSkidSteers)
{
  auto controller = bring_up();
  ASSERT_NE(controller, nullptr);

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
  EXPECT_NEAR(drive_command(0), 0.308, 1e-9);   // front_left
  EXPECT_NEAR(drive_command(1), 0.692, 1e-9);   // front_right
  EXPECT_NEAR(drive_command(2), 0.308, 1e-9);   // rear_left
  EXPECT_NEAR(drive_command(3), 0.692, 1e-9);   // rear_right
}

TEST_F(RP1SwerveControllerTest, Locked90IgnoresForwardAndCrabsWithFrontRearSplit)
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
    EXPECT_NEAR(steering_command(i), kPi / 2.0, 1e-9) << "corner " << i;
  }
  EXPECT_NEAR(drive_command(0), 0.62, 1e-9);  // front_left
  EXPECT_NEAR(drive_command(1), 0.62, 1e-9);  // front_right
  EXPECT_NEAR(drive_command(2), 0.38, 1e-9);  // rear_left
  EXPECT_NEAR(drive_command(3), 0.38, 1e-9);  // rear_right
}

TEST_F(RP1SwerveControllerTest, TwoWheelDefaultPairIsFrontFreeRearLocked)
{
  auto controller = bring_up();
  ASSERT_NE(controller, nullptr);

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
  EXPECT_NEAR(drive_command(2), 0.436, 1e-9);
  EXPECT_NEAR(drive_command(3), 0.564, 1e-9);
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
    EXPECT_NEAR(drive_command(i), 1.0, 1e-9) << "corner " << i;
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
