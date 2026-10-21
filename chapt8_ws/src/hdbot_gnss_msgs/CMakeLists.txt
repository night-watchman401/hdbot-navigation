cmake_minimum_required(VERSION 3.16)
project(hdbot_gnss_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DopplerObservation.msg" "msg/DopplerEpoch.msg"
  DEPENDENCIES std_msgs geometry_msgs)
ament_export_dependencies(rosidl_default_runtime)
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})
ament_package()
