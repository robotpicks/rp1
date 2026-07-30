cmake_minimum_required(VERSION 3.20)
project(rp1_swerve_controller)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(rp1_swerve_controller SHARED
  src/rp1_swerve_controller.cpp
)
target_compile_features(rp1_swerve_controller PUBLIC cxx_std_17)
target_include_directories(rp1_swerve_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Modern namespaced imported targets rather than ament_target_dependencies() -- see
# vesc_dronecan_ros/vesc_dronecan_driver/CMakeLists.txt for why (that macro is gone from current
# ament_cmake).
target_link_libraries(rp1_swerve_controller PUBLIC
  controller_interface::controller_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  tf2_ros::tf2_ros
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

pluginlib_export_plugin_description_file(controller_interface rp1_swerve_controller.xml)

install(
  TARGETS rp1_swerve_controller
  EXPORT export_rp1_swerve_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES rp1_swerve_controller.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(rp1_swerve_controller)
ament_export_targets(export_rp1_swerve_controller)
ament_export_dependencies(controller_interface pluginlib rclcpp rclcpp_lifecycle realtime_tools geometry_msgs nav_msgs std_msgs tf2_ros)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
