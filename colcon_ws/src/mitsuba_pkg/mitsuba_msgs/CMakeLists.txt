cmake_minimum_required(VERSION 3.8)
project(mitsuba_msgs)

# find dependencies
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/MitsubaState.msg"
)
#set(srv_files
#  "srv/MitsubaRtn.srv"
#)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MitsubaState.msg"
#  "srv/AddThreeInts.srv"

  DEPENDENCIES builtin_interfaces std_msgs
 )

ament_export_dependencies(rosidl_default_runtime)

ament_package()
