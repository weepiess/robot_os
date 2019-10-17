cmake_minimum_required(VERSION 3.1)
project(task_image_related)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  image_transport
  robot_msgs 

)

find_package(OpenCV 3 REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)



###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS include 
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS  roscpp cv_bridge image_transport robot_msgs
    DEPENDS OpenCV
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



## Build the task_image_related library
AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src DIR_SRCS)
add_library(${PROJECT_NAME} ${DIR_SRCS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} robot_msgs_generate_messages)

add_executable(task_show_image_node
  nodes/task_show_image_node.cpp
)
target_link_libraries(task_show_image_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)









