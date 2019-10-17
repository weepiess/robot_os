cmake_minimum_required(VERSION 3.1)
project(robot_msgs)


set(CMAKE_CXX_STANDARD 11)
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR})
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  actionlib
  message_generation
)

add_action_files(
  DIRECTORY action
  FILES
  ArmorDetection.action

)

add_message_files(
  DIRECTORY msg
  FILES
  ChassisInfo.msg
  RailChassisInfo.msg
  GimbalInfo.msg
  ShootInfo.msg
  AttackedInfo.msg
  TaskAutoAimInfo.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  SetMode.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package()
