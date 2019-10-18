cmake_minimum_required(VERSION 3.5)
project(robot_base_pr)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    robot_base
    robot_tool_pr
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS robot_base robot_tool_pr
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

## Build the robot_cam_uestc library
AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src DIR_SRCS)
add_library(${PROJECT_NAME} ${DIR_SRCS})
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

add_executable(serial_data_test_node
    nodes/serial_data_test_node.cpp
)
target_link_libraries(serial_data_test_node
    ${PROJECT_NAME}
)