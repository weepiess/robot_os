# robot_cam_uestc模块

## 1.简介

robot_cam_uestc是RoboMasterOS UESTC中的一个基础功能包，提供了工业相机ROS节点功能。主要实现功能如下：

- **工业相机ROS节点**

为了提高扩展能力，该模块将 **相机操作** 与 **相机ROS节点** 进行解偶，通过 **相机接口** 实现，所以，对于不同的工业相机，有不同的驱动，通过该 **相机接口** ，可以无需关心ROS部分，快速实现 **相机ROS节点** 。

## 2.文件说明

主要文件：

|           文件            |     功能描述     |
| :-----------------------: | :--------------: |
| mind_vision_cam_dev.h/cpp | 工业相机设备实现 |

node文件:

|           文件           |    功能描述     |
| :----------------------: | :-------------: |
| mind_vision_cam_node.cpp | 工业相机ROS节点 |

## 3.快速使用

__配置文件通用参数__：/res目录下

```yaml
cam_topic_name: "front_camera"
cam_fps: 30
```

- cam_topic_name：ROS发布图片topic节点名字。
- cam_fps：可以控制最大帧率，实际帧率小于等于该值。

### a.工业相机：

**修改配置文件**：res/mind_vision_cam_config.yaml

```yaml
enemy_is_red: 0
```

- enemy_is_red：敌方颜色，1为红色，0为蓝色

**修改相机参数配置文件**：res/mind_vision_blue_config.yaml     res/mind_vision_red_config.yaml

```yaml
frame_speed: 1
gamma: 24
exposure_time: 1000
contrast: 150
saturation: 100
resolution_width: 1280
resolution_height: 720
```

- frame_speed：帧率，0为普通模式，1为高速模式，2为超高速模式
- gamma：gamma值
- exposure_time：曝光时间
- contrast：对比度
- saturation：饱和度
- resolution_width：分辨率宽度
- resolution_height：分辨率高度

__运行：__

```bash
cd robot_cam_uestc/third_party/mind_vision

sudo ./install.sh

rosrun robot_cam_uestc mind_vision_cam_node
```

## 4.维护者及开源许可证

* gezp 1350824033@qq.com
* wyx 1418555317@qq.com

robot_cam_uestc provided under GPL-v3.