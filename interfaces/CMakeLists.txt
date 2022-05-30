cmake_minimum_required(VERSION 3.15)
project(interfaces)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


rosidl_generate_interfaces(interfaces
  "msg/CircleProjection.msg"
  "msg/CircleProjections.msg"
  "msg/Projection.msg"
  "msg/BoundingBoxAA.msg"
  "srv/ChangeProjection.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()