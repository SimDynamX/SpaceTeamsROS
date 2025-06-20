cmake_minimum_required(VERSION 3.5)
project(custom_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/StringToBool.srv"
  "srv/FloatToBool.srv"
)

ament_package()