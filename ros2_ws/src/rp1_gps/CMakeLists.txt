cmake_minimum_required(VERSION 3.20)
project(rp1_gps)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
