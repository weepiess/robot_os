# 安装使用

## １.安装ROS

参考官网[http://www.ros.org/](http://www.ros.org/)

由于ROS服务器在国外，下载速度慢，可以使用ustc镜像站下载：

__step1.设置sources.list__

```bash
sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list'
```

__step2.设置keys__

```bash
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
```

__step3.安装ros__

```bash
#更新
sudo apt-get update
#Kinetic发行版为例
sudo apt install ros-kinetic-desktop-full
#sudo apt install ros-kinetic-ros-base(可选，安装精简版)
```

> 发行版选择：
>     ubuntu14,ubuntu16：Kinetic
>     ubuntu18: Melodic


__step４.初始化rosdep__

```bash
sudo rosdep init
rosdep update
```

__step5.环境设置__

```bash
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

__step6.设置工作目录__

```bash
##建立工作目录
mkdir ~/catkin_ws
cd ~/catkin_ws
mkdir src
catkin_make
#加入环境
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 2.安装依赖项

安装Glog

```bash
sudo apt-get install libgoogle-glog-dev
```

## 3.使用RobotOS

__下载：__

在工作录下，使用git下载RoboMasterOS包

```bash
cd ~/catkin_ws/src
git clone https://gitlab.com/robomaster-os/rmos_public/robomaster_os.git
```

__编译：__

```bash
cd ~/catkin_ws
catkin_make
```

若无编译错误，即代表环境配置成功

