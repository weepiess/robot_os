////////////////////////////////////////////////////////////////////////////////
///Copyright(c)      UESTC ROBOMASTER2018      SerialPort Code for robot
///ALL RIGHTS RESERVED
///@file:serial_interface.h
///@brief: 机器人控制基本接口头文件，包含对车底盘及云台的基本接口。
///@vesion 1.0
///@author: gezp
///@email: 1350824033@qq.com
///@date: 18-3-4
///修订历史：
///2018.6.13:哨兵协议versio2.1
////////////////////////////////////////////////////////////////////////////////
#ifndef RMDEMO_SERIAL_INTERFACE_H
#define RMDEMO_SERIAL_INTERFACE_H

#include "robot_base/fixed_common_base.h"
#include "robot_base/serialport_trans_dev.h"

//协议
namespace sentry_base_uestc{

    typedef enum:unsigned char {
        //小主机和32之间的通信
        CMD_SERIAL_HEARTBEAT = 0x00,
        CMD_SERIAL_CHASSIS_STOP = 0x01,

        CMD_SERIAL_CHASSIS_POS_CONTROL = 0x02,
        CMD_SERIAL_CHASSIS_CONST_MOVE = 0x03,
        CMD_SERIAL_CHASSIS_RANDOM_MOVE = 0x04,
        CMD_SERIAL_CHASSIS_SWING_MOVE = 0x05,

        CMD_SERIAL_YUNTAI_RELATIVE_ANGLE = 0x10,
        CMD_SERIAL_YUNTAI_ABSOLUTE_ANGLE = 0x11,
        CMD_SERIAL_SHOOT = 0x12,
        CMD_SERIAL_YUNTAI_GLOBAL_SCAN = 0x13,
        CMD_SERIAL_YUNTAI_LOCAL_SCAN = 0x14,

        CMD_SERIAL_ENEMY_POSITION_SEND = 0x20,

        CMD_SERIAL_STATE_INFO_RECV = 0xb0,
        CMD_SERIAL_YUNTAI_ANGLE_INFO_RECV = 0xc0,

        //小主机和树莓派之间的通信
        CMD_SERIAL_ENEMY_INFO_UPDATE = 0xe0,

    } SerialPortCMD;


    class SerialInterface: public robot_base::FixedCommonBase{
    public:
        SerialInterface(void);
        ~SerialInterface(void);

    private:
        robot_base::SerialPortTransDev mSerialPortTransDev;

    public:
        /** 初始化函数
        *  @param:  std::string devPath :串口设备路径
        *  @return: int :错误号，0代表无错误，１代表发生错误。
        */
        int init(std::string devPath);

        /** 心跳包
        *  @param:  void
        *  @note :
        */
        bool heartBeat();
    };

    
};


#endif
