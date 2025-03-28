cmake_minimum_required(VERSION 3.5)
project(mitsuba_sample)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)

set(TG1 sample_node)
add_executable(${TG1} src/${TG1}.cpp)
ament_target_dependencies(${TG1} rclcpp sensor_msgs visualization_msgs can_msgs geometry_msgs nav_msgs tf2_ros pcl_ros)
install(TARGETS ${TG1} DESTINATION lib/${PROJECT_NAME})

#set(TG2 sample_node2)
#add_executable(${TG2} src/${TG2}.cpp)
#ament_target_dependencies(${TG2} rclcpp sensor_msgs visualization_msgs can_msgs geometry_msgs nav_msgs tf2_ros pcl_ros)
#install(TARGETS ${TG2} DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch rviz sdf yaml DESTINATION share/${PROJECT_NAME})

ament_package()
