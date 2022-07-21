cmake_minimum_required(VERSION 3.8)
project(ur_control)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ur_control_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ur_rtde 1.5.2 REQUIRED)

add_executable(ur_control src/ur_control.cpp)
target_compile_options(ur_control PRIVATE -Wall -Wextra)
ament_target_dependencies(ur_control rclcpp geometry_msgs sensor_msgs ur_control_msgs)
target_link_libraries(ur_control Boost::boost Eigen3::Eigen ur_rtde::rtde)

install(
  TARGETS ur_control
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
