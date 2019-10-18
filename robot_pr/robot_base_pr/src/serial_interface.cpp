////////////////////////////////////////////////////////////////////////////////
///Copyright(c)     UESTC ROBOMASTER2018      SerialPort Code for robot
///ALL RIGHTS RESERVED
///@file:serial_interface.cpp
///@brief: 机器人控制基本接口源文件，包含对车底盘及云台的基本接口。
///@vesion 1.0
///@author: gezp
///@email: 1350824033@qq.com
///@date: 18-3-4
///修订历史：
////////////////////////////////////////////////////////////////////////////////
#include "robot_base_pr/serial_interface.h"

using namespace robot_base;
using namespace sentry_base_uestc;

SerialInterface::SerialInterface(){}

SerialInterface::~SerialInterface() {}

int SerialInterface::init(std::string devPath){
    if(mSerialPortTransDev.init(devPath) == 0){
        setTransDev(&mSerialPortTransDev);
        return 0;
    }else{
        return -1;
    }
}

/*********************控制接口**************************/
bool SerialInterface::heartBeat(){
    FixedPacket packet;
    packet.loadData(CMD_SERIAL_HEARTBEAT,1);
    packet.pack();
    return sendPacket(packet) == 0;
}

