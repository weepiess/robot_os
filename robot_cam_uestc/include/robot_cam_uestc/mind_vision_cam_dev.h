#ifndef MIND_VISION_CAM_DEV_H
#define MIND_VISION_CAM_DEV_H

#include "CameraApi.h" //相机SDK的API头文件
#include "unistd.h"
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <iostream>

using namespace cv;
using namespace std;

#define CAMERA_GET_DATA 0
#define CAMERA_GET_OVERTIME -1
#define CAMERA_OFFLINE_ERROR -2

namespace robot_cam_uestc{

    class MindVisionWithoutThread{
    public:
        MindVisionWithoutThread();
        ~MindVisionWithoutThread();

    public:
        /** 相机初始化，直接获得句柄并让sdk开始工作（该情况只适用于只有一个相机的情况）
         * @return: int, 错误码，-1为错误，0为正确
         */
        int init(int image_width, int image_height,string path);

        /** 设置相机的分辨率
         * @param: width, 横向像素
         * @param: height, 纵向像素
         * @return: int, 错误码
         */
        int setResolution(int width, int height);

        /** 得到图像
         * @param: src, 得到的图像
         * @return: int, 返回值见相机返回值宏定义
         */
        int getImg(Mat &src);

        /** 根据传入的字符调整不同的参数
         * @param: c, 键盘输入的字符：
         *      "f"：frameSpeed  "g"：gamma  "e"：exposureTime  "c"：contrast，'n'：当前参数增大，'m'：当前参数减少，'s'，保存参数
         * @return: void
         */
        void adjustParams(unsigned char c);

    private:
        /** 初始化，内部调用
         */
        int init(string config_path);

    private:
        bool 		    isUnInit;
        unsigned char           * g_pRgbBuffer = NULL;       //处理后数据缓存区
        int                     iStatus=0;            //状态返回值
        int                     hCamera=-1;           //相机句柄
        tSdkCameraDevInfo       tCameraEnumList;      //设备列表信息
        tSdkCameraCapbility     tCapability;          //设备描述信息
        tSdkFrameHead           sFrameInfo;           //每一帧的头指针
        BYTE*			        pbyBuffer;            //数据缓冲区
        IplImage                *iplImage = NULL;     //图像的矩阵存储形式     
        int                     channel=3;            //通道数
        tSdkImageResolution     tResolution;          //像素
        string                  mPath;
        bool is_enemy_red; //敌方颜色，用于重连时初始化

        int camera_get_overtime_times; //记录未读取到相机的次数，用于重启相机

        int index, gamma, exposure_time, contrast, saturation, resolution_width, resolution_height; //相机参数

        /** 当前的操作是作用在哪个相机元素上
         * "f"：frameSpeed  "g"：gamma  "e"：exposureTime  "c"：contrast 'b'：saturation
         */
        unsigned char curr_operation;
    };

};
#endif
