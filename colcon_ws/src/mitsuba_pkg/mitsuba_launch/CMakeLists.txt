cmake_minimum_required(VERSION 3.5)
project(mitsuba_launch)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch map rviz urdf yaml DESTINATION share/${PROJECT_NAME})

ament_package()
