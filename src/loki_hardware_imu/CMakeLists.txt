cmake_minimum_required(VERSION 3.8)
project(loki_hardware_imu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(loki_msgs REQUIRED)

add_executable(ahrs_orientation
  src/ahrs_orientation.cpp
)

target_include_directories(ahrs_orientation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ahrs_orientation
  rclcpp
  sensor_msgs
  loki_msgs
)

install(TARGETS
  ahrs_orientation
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()