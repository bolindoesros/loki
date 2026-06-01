cmake_minimum_required(VERSION 3.8)
project(esp32_bench)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(auv_interfaces REQUIRED)

add_executable(ros_to_esp src/ros_to_esp.cpp)
ament_target_dependencies(ros_to_esp rclcpp auv_interfaces)

install(TARGETS
  ros_to_esp
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()