cmake_minimum_required(VERSION 3.1)
project(ur_control)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    geometry_msgs
    sensor_msgs
    ursurg_common
    ur_control_msgs
)
find_package(Eigen3 REQUIRED)
find_package(ur_rtde 1.2.3 REQUIRED)

catkin_package()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)

if((CMAKE_CXX_COMPILER_ID MATCHES "Clang") OR (CMAKE_CXX_COMPILER_ID MATCHES "GNU"))
  add_compile_options(-Wall -Wextra)
endif()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(ur_control
  src/ur_control.cpp
  src/monotonic.cpp
)

target_link_libraries(ur_control
  Eigen3::Eigen
  ur_rtde::rtde
  ${catkin_LIBRARIES}
)
