#ifndef KALMAN_FILTER_BY_OPENCV_H
#define KALMAN_FILTER_BY_OPENCV_H

#include <opencv2/opencv.hpp>

namespace robot_tool {

class SimpleKalmanFilter{
public:
    SimpleKalmanFilter();
    ~SimpleKalmanFilter();
public:
    //以下函数均封装了KFilter的相同函数，外部调用时可直接调用以下函数，简化程序
    void init(int stateDimens, int measureDimens, float processNoiseCov, float measureNoiseCov, float errorCovPost,
        int controlDimens = 0, int type = CV_32F);
    //调用此函数前需对measurement进行赋值，返回滤波过后的状态向量
    const cv::Mat& correct();
    //根据前一时刻的状态对此刻进行预测
    const cv::Mat& predict();
public:
    //opencv自带的卡尔曼类
    cv::KalmanFilter kFilter;
    //观测值矩阵
    cv::Mat measurement;
};
};


#endif

/* 关于调参的建议:
Q值为过程噪声，越小系统越容易收敛，我们对模型预测的值信任度越高；但是太小则容易发散，如果Q为零，那么我们只相信预测值；
Q值越大我们对于预测的信任度就越低，而对测量值的信任度就变高；如果Q值无穷大，那么我们只信任测量值；
R值为测量噪声，太小太大都不一定合适。R太大，卡尔曼滤波响应会变慢，因为它对新测量的值的信任度降低；越小系统收敛越快，但过小则容易出现震荡；

测试时可以先将Q从小往大调整，将R从大往小调整；先固定一个值去调整另外一个值，看收敛速度与波形输出。

系统中还有一个关键值P，它是误差协方差初始值，表示我们对当前预测状态的信任度，它越小说明我们越相信当前预测状态；
它的值决定了初始收敛速度，一般开始设一个较大的值以便于获取较快的收敛速度。
随着卡尔曼滤波的迭代，P的值会不断的改变，当系统进入稳态之后P值会收敛成一个最小的估计方差矩阵，这个时候的卡尔曼增益也是最优的，所以这个值只是影响初始收敛速度。
*/


/*
卡尔曼滤波预测的使用流程：
1. 声明变量并初始化:
SimpleKalmanFilter skf;
skf.init(2, 1, 1e-1, 1e-5, 1e-2);
skf.KFilter.transitionMatrix = (Mat_<float>(2, 2) << 1, dt, 0, 1); //转移矩阵初始化（必须）
skf.KFilter.statePost = (Mat_<float>(2, 1) << 当前状态值); //初始状态的初始化（必须）

2. 调用skf.predict()进行一步预测，获得预测后的数据（如果需要动态改变采样时间，在调用predict前需要重新初始化转移矩阵）

3. 获取当前的观测值，更新观测值矩阵，并调用skf.correct()获得滤波后的值
skf.measurement.at<float>(0) = 观测值;
const Mat& filter_state = skf.correct();

4. 根据需要进行预测，可以用滤波后的状态值中的速度值乘以采样时间，加在当前滤波后的状态上，作为预测后的状态
*/