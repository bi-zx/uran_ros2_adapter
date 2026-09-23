cmake_minimum_required(VERSION 3.8)
project(uran_ros2_msgs VERSION 1.0.0)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ContractEnvelope.msg"
  "msg/OperationEvent.msg"
  "srv/RuntimeAdmin.srv")
ament_export_dependencies(rosidl_default_runtime)
ament_package()
