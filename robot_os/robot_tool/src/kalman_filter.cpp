#include "robot_tool/kalman_filter.h"

using namespace std;
using namespace cv;
using namespace robot_tool;

SimpleKalmanFilter::SimpleKalmanFilter(){}
SimpleKalmanFilter::~SimpleKalmanFilter(){}

void SimpleKalmanFilter::init(int stateDimens, int measureDimens, float processNoiseCov, float measureNoiseCov,
        float errorCovPost, int controlDimens, int type){
    //对Kalman初始化
    kFilter.init(stateDimens, measureDimens, controlDimens, type);
    //对各项参数初始化
    setIdentity(kFilter.processNoiseCov, Scalar::all(processNoiseCov)); //过程噪声
    setIdentity(kFilter.measurementNoiseCov, Scalar::all(measureNoiseCov)); //测量噪声
    setIdentity(kFilter.errorCovPost, Scalar::all(errorCovPost)); //误差协方差
    setIdentity(kFilter.measurementMatrix, Scalar::all(1)); //原始测量矩阵被初始化为0，这里必须初始化为1才能正确收敛
    //初始化观测值矩阵
    measurement = Mat::zeros(measureDimens, 1, type);
}

const Mat& SimpleKalmanFilter::correct(){
    return kFilter.correct(measurement);
}

const Mat& SimpleKalmanFilter::predict(){
    return kFilter.predict();
}