# TEMPORARY: only the 3 messages S2 needs, copied field-for-field from
# assignment section 6. Delete this package once M5's full
# smart_hub_interfaces exists, and use theirs instead.
cmake_minimum_required(VERSION 3.8)
project(smart_hub_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/QueueState.msg"
  "msg/DemandPrediction.msg"
  "msg/HubEvent.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
