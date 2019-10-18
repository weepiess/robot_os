#include "ros/ros.h"
#include "robot_base_pr/serial_interface.h"
#include "robot_tool_pr/function_thread.h"

#include <signal.h>
#include "ros/package.h"

using namespace robot_base;
using namespace sentry_base_uestc;
using namespace sentry_tool_uestc;

//定义全局串口变量
SerialInterface send_serial_interface;
SerialInterface receive_serial_interface;

void SignalHandler(int signal){
    if(ros::isInitialized() && ros::isStarted() && ros::ok() && !ros::isShuttingDown()){
        ros::shutdown();
    }
}

void handleSendFunction(){
    //发送帧
    const int MAX_SEND_LENGTH = 1000;
    int send_error_frames = 0;
    for(int i=1; i<=MAX_SEND_LENGTH; i++){
        int send_success = send_serial_interface.heartBeat();
        if(!send_success){
            send_error_frames++;
            std::cout<<"fucking error!!\n";
        }else{
            std::cout<<"send success !\n";
        }
        usleep(1500);
    }
    //统计发送帧情况
    std::cout<<"send error frames: "<<send_error_frames<<std::endl;
}

void handleReceiveFunction(){
    //接收帧
    const int MAX_RECEIVE_LENGTH = 500;
    FixedPacket packet;
    int dev_error_frames = 0, check_error_frames = 0, success_frames = 0;
    for(int i=1; i<=MAX_RECEIVE_LENGTH; i++){
        int result = receive_serial_interface.recvPacket(packet);
        unsigned char CMD;
        packet.unloadData(CMD, 1);
        std::cout<<"receving command: "<<(int)CMD<<std::endl;
        if(result == 0){
            success_frames++;
            //std::cout<<"succss loading!\n";
        }
        else if(result == -1)
            check_error_frames++;
        else if(result == -2)
            dev_error_frames++;
    }

    //统计收到帧的情况
    std::cout<<"success frames: "<<success_frames
        <<"\ndev error frames: "<<dev_error_frames
        <<"\ncheck error frames: "<<check_error_frames
        <<"\ntotal receive frames: "<<(success_frames+dev_error_frames+check_error_frames)<<std::endl;
}

int main(int argc, char **argv){
    signal(SIGINT, SignalHandler);
    signal(SIGTERM,SignalHandler);
    ros::init(argc, argv, "serial_send_and_receive_test_node",ros::init_options::NoSigintHandler);

    //初始化串口模块
    if(send_serial_interface.init("/dev/ttyUSB0") != 0 || receive_serial_interface.init("/dev/ttyUSB0") != 0)
        return EXIT_FAILURE;

    //创建串口发送和接收线程
    FunctionThread send_thread;
    send_thread.init(&handleSendFunction);

    FunctionThread receive_thread;
    receive_thread.init(&handleReceiveFunction);

    //启动线程，开始发送和接收
    receive_thread.start();
    send_thread.start();
    
    ros::AsyncSpinner async_spinner(1);
    async_spinner.start();
  
    ros::waitForShutdown();
    return EXIT_SUCCESS;
}