cmake_minimum_required(VERSION 3.5)
project(robot_tool_pr)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
    robot_tool
)
find_package(Glog REQUIRED)
find_package(OpenCV 3.4.3 REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS robot_tool
   DEPENDS GLOG
)

include_directories(
    include
    ${GLOG_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

## Build the robot_cam_uestc library
AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src DIR_SRCS)
add_library(${PROJECT_NAME} STATIC ${DIR_SRCS})

target_link_libraries(${PROJECT_NAME}
    pthread
    ${catkin_LIBRARIES}
    ${GLOG_LIBRARIES}
)
