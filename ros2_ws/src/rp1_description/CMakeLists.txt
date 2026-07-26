cmake_minimum_required(VERSION 3.20)
project(rp1_description)

find_package(ament_cmake REQUIRED)

# Descriptions only -- nothing compiles here. The hardware component these URDFs name lives in
# the vesc_dronecan_ros submodule (package vesc_dronecan_driver), which is robot-agnostic; what
# is rp1-specific is the geometry and the DroneCAN id assignments in these files.
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
