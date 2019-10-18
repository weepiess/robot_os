cmake_minimum_required(VERSION 3.5)
project(robot_core)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    robot_base_pr
    robot_tool_pr
    robot_tool
)

find_package(TensorflowCC REQUIRED COMPONENTS Static)

catkin_package(
    CATKIN_DEPENDS robot_tool
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}

    ${PROJECT_SOURCE_DIR}/libBT/include
    ${PROJECT_SOURCE_DIR}/libSentryDecision/include
)

add_subdirectory(${PROJECT_SOURCE_DIR}/libBT)
add_subdirectory(${PROJECT_SOURCE_DIR}/libSentryDecision)

aux_source_directory(${PROJECT_SOURCE_DIR}/src DIR_SRCS)
# add_executable(${PROJECT_NAME} ${DIR_SRCS})

# target_link_libraries(${PROJECT_NAME}
#     SentryDecision
#     ${catkin_LIBRARIES}
# )
