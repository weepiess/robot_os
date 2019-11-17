#include "control_model.h"
#include "serial_listen_thread.h"
#include "serial_port_debug.h"
#include "base_aim.h"
#include "fstream"
#include "mind_vision_without_thread.h"
#include "time.h"
#include <iostream>
#include <string>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>

using namespace std;

bool enemy_red;
int img_width, img_height;
string now_date, now_time;
cv::Mat pic;

// GET SYS TIME
string getTime()
{
    struct timeval tv;
    gettimeofday(&tv,NULL);
    struct tm* pTime;
    pTime = localtime(&tv.tv_sec);

    char sTemp[40] = {0};
    snprintf(sTemp, sizeof(sTemp), "%04d-%02d-%02d-%02d-%02d-%02d-%03d-%03d", pTime->tm_year+1900, \
    pTime->tm_mon+1, pTime->tm_mday, pTime->tm_hour, pTime->tm_min, pTime->tm_sec, \
    tv.tv_usec/1000,tv.tv_usec%1000);
    return (string)sTemp;
}

int main()
{
    while (true)
    {
        cv::FileStorage f;
        f = FileStorage("../res/mian_config.yaml", cv::FileStorage::READ);
            f["enemy_red"] >> enemy_red;
            f["height"] >> img_height;
            f["width"] >> img_width;
            f.release();
        cout<<img_height<<"    "<<img_width;
        MindVisionWithoutThread MVWT;
        // cout<<"asdf";
        // waitKey();
        if(MVWT.init(enemy_red,img_height,img_width)==0)
        {
            cout<<"\nOK\n";
        }
        else
        {
            continue;
        }
        
        //char str;
        while (true)
            {
                if (true){
                    cout<<"!!!!!!!!\n";
                    if (MVWT.getImg(pic) == 0)
                    {
                        string temp = getTime();
                        cv::imwrite("../../pic_save/"+temp+".jpg",pic);
                    }
                    else if (MVWT.getImg(pic) == -1)
                    {
                        continue;
                    }
                    else if (MVWT.getImg(pic) == -2)
                    {
                        break;
                    }
                }
                else
                {
                    break;
                }
                
            }
        string cc;
        cin >> cc;
        if (cc == "Q"){
            break;
        }
    }
}


//     int MindVisionWithoutThread::init(bool is_enemy_red_, int image_width, int image_height){
//     is_enemy_red = is_enemy_red_;
//     resolution_width = image_width;
//     resolution_height = image_height;

//     if(init() != 0)
//         return -1;
//     cout<<"camera is ready, now open it and capture.";
//     return 0;
// }


    // RobotModel robotModel;
    // cout<<"[robot init]robot model start to initialize!"<<endl;
    // robotModel.init();
    // cout<<"[robot init]robot control model start to initializ!"<<endl;
    // usleep(10000);
    // ControlModel controlModel;
    // controlModel.init(&robotModel);
    // cout<<"[robot init]robot serial port start to listen!"<<endl;
    // SerialListenThread serialListenThread;
    // serialListenThread.init(&robotModel,&controlModel);

    // cout<<"[robot init]robot init end!"<<endl;
    // //debug模块
    // // SerialPortDebug serialPortDebug;
    // // serialPortDebug.init(robotModel.getpSerialInterface());
    // // serialPortDebug.testSerialPort();
    // //主逻辑
    // while(true){
    //     controlModel.processFSM();
    // }
    // serialListenThread.join();
    // cout<<"error end!"<<endl;
    // getchar();//防止监听线程意外结束直接退出。
    // return 0;

