cmake_minimum_required(VERSION 3.5)
project(robot_cam_uestc)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 11)
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    robot_cam
    robot_tool
)
find_package(MVSDK REQUIRED)

catkin_package(
   INCLUDE_DIRS include ${MVSDK_INCLUDE_DIRS}
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS robot_cam
   CATKIN_DEPENDS robot_tool
)

include_directories(
    include
    ${MVSDK_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

## Build the robot_cam_uestc library
AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src DIR_SRCS)
add_library(${PROJECT_NAME} ${DIR_SRCS})
target_link_libraries(${PROJECT_NAME}
    ${MVSDK_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_executable(mind_vision_cam_node
    nodes/mind_vision_cam_node.cpp
)
target_link_libraries(mind_vision_cam_node
    ${PROJECT_NAME}
)

add_executable(mind_vision_cam_thread_node
    nodes/mind_vision_cam_thread_node.cpp
)
target_link_libraries(mind_vision_cam_thread_node
    ${PROJECT_NAME}
)

add_executable(usb_cam_thread_example
    nodes/usb_cam_thread_example.cpp
)
target_link_libraries(usb_cam_thread_example
    ${PROJECT_NAME}
)
