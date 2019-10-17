cmake_minimum_required(VERSION 3.5)
project(robot_cam)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV 3 REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
    INCLUDE_DIRS include 
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS  roscpp cv_bridge image_transport
    DEPENDS OpenCV
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Build the robot_cam library
AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src DIR_SRCS)
add_library(${PROJECT_NAME} ${DIR_SRCS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)




add_executable(usb_cam_node
  nodes/usb_cam_node.cpp
)
target_link_libraries(usb_cam_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)


add_executable(sim_cam_image_node
  nodes/sim_cam_image_node.cpp
)
target_link_libraries(sim_cam_image_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

add_executable(sim_cam_video_node
  nodes/sim_cam_video_node.cpp
)
target_link_libraries(sim_cam_video_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)







