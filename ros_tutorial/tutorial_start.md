# ROS_tutorial start

#### author by :fwc

---

## 0x01 什么是catkin_make

程序在cmake编译是这样的流程, cmake指令依据你的CMakeLists.txt 文件,生成makefiles文件,make再依据此makefiles文件编译链接生成可执行文件. catkin_make是将cmake与make的编译方式做了一个封装的指令工具, 规范了工作路径与生成文件路径.

### cmake标准流程

![](/home/weepies/catkin_ws/src/ros_tutorial/res/pic.png)

~~~ shell
mkdir build
cd build
cmake ..
make 
~~~



### catkin_make标准流程

ROS 的 Catkin 编译系统的一个特点是将程序做成 package (称为 catkin package 或者 ROS package) 的形式，可以理解成模块化。典型的 ROS workspace 中包含 src, build, devel 三个文件夹，在分享时只需要分享 src 中的某个 package 即可，所有的编译信息都在此 package 中。一个 package 在编译时可以指定依赖于另一个 package。

~~~ shell
cd /工作空间
catkin_make
# or: catkin_make “指定的包”

#可选：
catkin_make install
~~~



## 0x02 ROS 包基本架构：

~~~ shell
/xxx package
	/src
		xxx.cpp
	/include
		/xxx package
			xxx.h
	CMakeLists.txt
	package.xml
~~~



### 如何编写CMakelists.txt:

​     所需CMake版本（cmake_minimum_required）

​     软件包名称（project（））

​     查找构建所需的其他CMake / Catkin软件包（find_package（））

​     启用Python模块支持（catkin_python_setup（））## 可选

​     消息/服务/动作生成器（add_message_files（），add_service_files（），add_action_files（））## 可选

​     调用消息/服务/动作生成（generate_messages（））## 可选

​     指定package build info export（catkin_package（））

​     要建立的库/可执行文件（add_library（）/ add_executable（）/ target_link_libraries（））

​     测试建立（catkin_add_gtest（））## 可选

​     安装规则（install（）） ##可选

### 关于package.xml

package.xml指明 package 在编译和运行时依赖于哪些其他 package，同时也包含该 package 的一些描述信息，如作者、版本等.

~~~ xml
<?xml version="1.0"?>
<package format="2">
  <name>beginner_tutorials</name>
  <version>0.0.0</version>
  <description>The beginner_tutorials package</description>
  <maintainer email="weepies@todo.todo">weepies</maintainer>

  <license>TODO</license>


  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>


  <!-- The export tag contains other, unspecified, tags -->
  <export>
    <!-- Other tools can request additional information be placed here -->

  </export>
</package>
~~~



### 搭建第一个ros项目：

~~~ shell
cd /${your workshop}/src/
mkdir ros_tutorial
cd /ros_tutorial
mkdir include
mkdir src
touch CMakeLists.txt
touch package.xml
cd include
touch test.h
cd ../src
touch test.cpp
~~~

test.cpp:

~~~c++
#include "test.h"

int main(){
    std::cout<<"hello world!!\n";
}
~~~

test.h:

~~~ c++
#ifndef TEST_H
#define TEST_H

#include "iostream"

int a;

#endif
~~~



CMakeLists.txt:

~~~ cmake
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
~~~



package.xml:

~~~ xml
<?xml version="1.0"?>
<package format="2">
  <name>ros_tutorial</name>
  <version>0.1.0</version>
  <description>The robot base controller package</description>
  <!-- One maintainer tag required, multiple allowed, one person per tag -->
  <!-- Example:  -->
  <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
  <author>FWC</author>
  <maintainer email="2383226319@qq.com">FWC</maintainer>

  <!-- One license tag required, multiple allowed, one license per tag -->
  <!-- Commonly used license strings: -->
  <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
  <license>GPL 3.0 </license>
  <!-- Url tags are optional, but multiple are allowed, one per tag -->

  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>robot_msgs</build_depend>

  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>robot_msgs</exec_depend>
  <!-- The export tag contains other, unspecified, tags -->
  <export>
    <!-- Other tools can request additional information be placed here -->

  </export>

</package>

~~~



~~~ shell
cd /${your workshop}
catkin make
rosrun rosrun ros_tutorial test_example
~~~

