cmake_minimum_required(VERSION 3.5)
project(mitsuba_diff_drive)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(mitsuba_msgs REQUIRED)

install(PROGRAMS
  src/logging
  src/can_to_odo
  src/cmd_to_can
  src/pi_to_can
  src/duty_to_can
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY src/errorlist DESTINATION share/${PROJECT_NAME})

install(DIRECTORY DESTINATION share/${PROJECT_NAME})

ament_package()

