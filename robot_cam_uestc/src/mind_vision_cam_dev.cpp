#include "robot_cam_uestc/mind_vision_cam_dev.h"

using namespace robot_cam_uestc;
//#define RECEIVE_IMG_TIME_COUNT

MindVisionWithoutThread::MindVisionWithoutThread(){
    //sdk初始化
    isUnInit = false;
    CameraSdkInit(1);
}

MindVisionWithoutThread::~MindVisionWithoutThread(){
    if(hCamera != -1 && CameraConnectTest(hCamera) == 0)
        CameraUnInit(hCamera);
    if(g_pRgbBuffer != NULL){
        free(g_pRgbBuffer);
        g_pRgbBuffer = NULL;
    }
}

int MindVisionWithoutThread::init(int image_width, int image_height, string path){
    resolution_width = image_width;
    resolution_height = image_height;
    mPath = path;
    if(init(path) != 0)
        return -1;
    cout<<"camera is ready, now open it and capture.";
    return 0;
}

int MindVisionWithoutThread::init(string config_path){
    //枚举设备，并建立设备列表
    int iCameraCounts = 1;
    CameraEnumerateDevice(&tCameraEnumList, &iCameraCounts);
    //没有连接设备
    if(iCameraCounts==0){
        cout<<"No avaliable camera!!!";
        return -1;
    }
    
    //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    int result = CameraInit(&tCameraEnumList,-1,-1,&hCamera);
    cout<<"result: "<<result<<"\n";
    if(hCamera == -1){
        cout<<"Camera init failed!!!";
        return -1;
    }
    //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CameraGetCapability(hCamera, &tCapability);

    g_pRgbBuffer = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);
    /*让SDK进入工作模式，开始接收来自相机发送的图像数据。如果当前相机是触发模式，则需要接收到触发帧以后才会更新图像。*/
    iStatus = CameraPlay(hCamera);

    if(iStatus != 0){
        cout<<"sdk play failed!!!";
        return -1;
    }
    /*其他的相机参数设置
    例如 CameraSetExposureTime   CameraGetExposureTime  设置/读取曝光时间
         CameraSetImageResolution  CameraGetImageResolution 设置/读取分辨率
         CameraSetGamma、CameraSetConrast、CameraSetGain等设置图像伽马、对比度、RGB数字增益等等。
         更多的参数的设置方法，，清参考MindVision_Demo。本例程只是为了演示如何将SDK中获取的图像，转成OpenCV的图像格式,以便调用OpenCV的图像处理函数进行后续开发
    */
    cv::FileStorage f;
    f = FileStorage(config_path, cv::FileStorage::READ);


    f["frame_speed"] >> index;
    f["gamma"] >> gamma;
    f["exposure_time"] >> exposure_time;
    f["contrast"] >> contrast;
    f["saturation"] >> saturation;
    f.release();

    //帧率
    CameraSetFrameSpeed(hCamera, index);
    //gamma值
    CameraSetGamma(hCamera, gamma);
    //手动曝光
    CameraSetAeState(hCamera, FALSE);
    //曝光时间
    CameraSetExposureTime(hCamera, exposure_time);
    //对比度
    CameraSetContrast(hCamera, contrast);
    //白平衡
    CameraSetWbMode(hCamera, TRUE);
    //饱和度
    CameraSetSaturation(hCamera, saturation);
    //色温
    //CameraSetPresetClrTemp(hCamera, 0);
    //色温增益
    //CameraSetClrTempMode(hCamera, 1);

    //设置分辨率，默认是1280*1024
    if(setResolution(resolution_width, resolution_height) == -1){
        cout<<"resolution width --- 1280, resolution height --- 1024.";
        setResolution(1920, 1200);
    }

    if(tCapability.sIspCapacity.bMonoSensor){
        channel=1;
        iStatus = CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_MONO8);
    }else{
        channel=3;
        iStatus = CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_BGR8);
    }
    if(iStatus != 0){
        cout<<"set camera out-format error.";
        return -1;
    }
}

int MindVisionWithoutThread::setResolution(int width, int height){
    CameraGetImageResolution(hCamera, &tResolution);
    tResolution.iIndex = 0xFF;
    tResolution.iWidth = width;
    tResolution.iHeight = height;
    tResolution.iWidthFOV = width;
    tResolution.iHeightFOV = height;
    if(tResolution.iWidth <= 1920)
        tResolution.iHOffsetFOV = (1920 - tResolution.iWidth) / 2;
    else 
        tResolution.iHOffsetFOV = 0;
    if(tResolution.iHeight <= 1200)
        tResolution.iVOffsetFOV = (1200 - tResolution.iHeight) / 2;
    else 
        tResolution.iVOffsetFOV = 0;
    if(CameraSetImageResolution(hCamera, &tResolution) != 0){
        cout<<"set image's resolution error!!!";
        return -1;
    }
    resolution_width = width;
    resolution_height = height;
    usleep(5000); //休眠5ms等待分辨率设置好
    return 0;
}

int MindVisionWithoutThread::getImg(Mat& src){
    #ifdef RECEIVE_IMG_TIME_COUNT
    auto get_raw_img_start = std::chrono::steady_clock::now();
    #endif

    if(CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 5000) == CAMERA_STATUS_SUCCESS && g_pRgbBuffer != NULL){
        camera_get_overtime_times = 0;
        CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer, &sFrameInfo);
        if (iplImage){
            cvReleaseImageHeader(&iplImage);
        }
        iplImage = cvCreateImageHeader(cvSize(sFrameInfo.iWidth,sFrameInfo.iHeight),IPL_DEPTH_8U,channel);
        cvSetData(iplImage,g_pRgbBuffer,sFrameInfo.iWidth*channel);//此处只是设置指针，无图像块数据拷贝，不需担心转换效率

        if(iplImage != NULL && iplImage->height == resolution_height && iplImage->width == resolution_width){
            src = cv::cvarrToMat(iplImage, false); //转为Mat类型

            #ifdef RECEIVE_IMG_TIME_COUNT
            auto get_raw_img_end = std::chrono::steady_clock::now();
            std::chrono::microseconds duration = std::chrono::duration_cast<std::chrono::microseconds>(get_raw_img_end - get_raw_img_start);
            std::cout<<"getRawImage() time cost: "<<duration.count()<<std::endl;
            #endif

            //在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
            //否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
            CameraReleaseImageBuffer(hCamera,pbyBuffer);
            return CAMERA_GET_DATA;
        }
        CameraReleaseImageBuffer(hCamera,pbyBuffer);
    }

    if(!isUnInit){
        if(hCamera != -1 && CameraConnectTest(hCamera) == 0)
            CameraUnInit(hCamera);
        if(g_pRgbBuffer != NULL){
            free(g_pRgbBuffer);
            g_pRgbBuffer = NULL;
        }
	    isUnInit = true;
    } else {
        if(init(mPath) == 0){
            sleep(2);
            isUnInit = false;
        }
    }

    if(CameraConnectTest(hCamera) != 0){ //相机掉线，重启程序
        return CAMERA_OFFLINE_ERROR;
    } else { //相机未掉线
        return CAMERA_GET_OVERTIME;
    }

}

void MindVisionWithoutThread::adjustParams(unsigned char c){
    if(c=='f' || c=='g' || c=='c' || c=='a' || c=='e' || c=='b'){
        curr_operation = c;
        return;
    }
    if(!(c == 'n' || c == 'm' || c == 's')) return;
    cout<<c<<endl;
    //保存参数
    if(c=='s'){
        cv::FileStorage f;
        f = cv::FileStorage(mPath, cv::FileStorage::WRITE);
        f << "frame_speed" << index;
        f << "gamma" << gamma;
        f << "exposure_time" << exposure_time;
        f << "contrast" << contrast;
        f << "saturation" << saturation;
        f.release();
        return;
    }
    switch(curr_operation){
        case 'f':{
            if(c == 'n' && index < 2) ++index;
            if(c == 'm' && index > 0) --index;
            CameraSetFrameSpeed(hCamera, index);
            std::cout<<index<<std::endl;
            break;
        }
        case 'g':{
            if(c == 'n') gamma+=2;
            if(c == 'm' && gamma>=2) gamma-=2;
            CameraSetGamma(hCamera, gamma);
            std::cout<<gamma<<std::endl;
            break;
        }
        case 'c':{
            if(c == 'n')  contrast+=2;
            if(c == 'm' && contrast>=2) contrast-=2;
            CameraSetContrast(hCamera, contrast);
            std::cout<<contrast<<std::endl;
            break;
        }
        case 'e':{
            if(c == 'n') exposure_time+=100;
            if(c == 'm' && exposure_time>=100) exposure_time-=100;
            CameraSetExposureTime(hCamera, exposure_time);
            std::cout<<exposure_time<<std::endl;
            break;
        }
        case 'b':{
            if(c == 'n') saturation+=2;
            if(c == 'm' && saturation>=2) saturation-=2;
            CameraSetSaturation(hCamera, saturation);
            std::cout<<saturation<<std::endl;
            break;
        }
    }
}
