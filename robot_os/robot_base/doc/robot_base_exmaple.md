# robot_base自定义开发

### 1.fixed_common_base

robot_base_example基于fixed_common_base开发，大大方便了开发流程。首先可以了解一下fixed_common_base的API。

```c++
//设置发送设备，需要初始化的时候，传入设备指针
void setTransDev(TransDevInterface* trans_dev);
//发送设备是否打开
bool isOpen();
//数据包收发，收发成功返回0，否则返回其他
int sendPacket(FixedPacket packet);
int recvPacket(FixedPacket &packet);
```

在进行二次开发时，可以充分利用这几个接口。现在有两种方式去使用这些接口。

* 实例化。即创建一个FixedCommonBase对象，通过调用对象的这几个方法，来使用这些接口。
* 继承。即继承FixedCommonBase类，可以通过调用父类的方法，来使用这些接口。

在robot_base_example，使用的是__继承__方法。

### 2.数据包发送

* 基于ROS Topic开发，当收到gimbal控制Topic时，将topic中的消息转化为FixedPacket，再使用sendPacket()函数发送即可。

```c++
void RobotBaseExample::gimbalCallback(const robot_msgs::GimbalInfo & info)
{
    FixedPacket packet;
    packet.loadData<unsigned char  (protocol_example::Gimbal_Angle_Control,1);
    packet.loadData<unsigned char>(0x00,2);
    packet.loadData<float>(info.pitch_angle,3);
    packet.loadData<float>(info.yaw_angle,7);
    packet.pack();
    sendPacket(packet);
    //delay for data send.
    std::this_thread::sleep_for( std::chrono::milliseconds(5));
}
```

### 3.数据包接收

* 采用轮询接收方式，需要单独的线程去监听设备，这里实现了listenDev函数，将在主函数中被周期循环调用，实现监听效果。
* listenDev()中使用recvPacket()函数进行包接收，然后进行包中的数据处理。

```c++
void RobotBaseExample::listenDev(){
    FixedPacket packet;
    if(recvPacket(packet)==0){
        //the packet have already unpacked.
        unsigned char cmd;
        packet.unloadData(cmd,1);
        if(cmd==(unsigned char)protocol_example::Change_Mode){
            unsigned char mode=0;
            packet.unloadData(mode,2);
            if(mode==0x00){
                ROS_INFO("change mode: normal mode");
            }else if(mode==0x01){
                ROS_INFO("change mode: auto aim mode");
            }else{
                ROS_INFO("change mode:  mode err!");
            }
        }else{

        }
    }
}
```

