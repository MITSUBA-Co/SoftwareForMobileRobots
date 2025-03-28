cmake_minimum_required(VERSION 3.5)
project(mitsuba_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch sdf yaml DESTINATION share/${PROJECT_NAME})

ament_package()
