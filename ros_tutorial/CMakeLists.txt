cmake_minimum_required(VERSION 3.1)
project(ros_tutorial)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV 3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
# catkin_package(
#    INCLUDE_DIRS include
#    LIBRARIES ${PROJECT_NAME}
#    CATKIN_DEPENDS roscpp robot_msgs
# )

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp
)

###########
## Build ##
###########
include_directories(
  include ##该参数表示 包中的include目录，也是路径的一部分
  ${catkin_INCLUDE_DIRS}
)


#ros_tutorial lib
AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src DIR_SRCS) ## 查找指定源文件，然后存入相应的变量名
add_library(${PROJECT_NAME} ${DIR_SRCS}) ## 构建库
target_link_libraries(${PROJECT_NAME} ##链接其他库
  ${catkin_LIBRARIES}
)


## example_base_node

add_executable(test_example src/test.cpp)

target_link_libraries(test_example
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${PROJECT_NAME}
)