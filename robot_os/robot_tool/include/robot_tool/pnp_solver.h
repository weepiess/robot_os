#ifndef PNP_SOLVER_H
#define PNP_SOLVER_H

#include <opencv2/opencv.hpp>
#include <iostream>

namespace robot_tool {

class PNPSolver{
public:
    PNPSolver();
    ~PNPSolver();
public:
    //设置相机参数矩阵
    void setCameraMatrix(float fx, float fy, float cx, float cy);
    /** 设置相机畸变矩阵
     * 四个参数分别为x,y径向和切向畸变
     */
    void setDistortionCoef(float rdx, float rdy, float tdx, float tdy);
    /** 压入三维世界坐标点
     */
    void pushPoints3D(const cv::Point3d& point);
    /** 压入三维世界坐标点, 默认以传入正方体的中心作为坐标原点，并且以xy平面的顺时针方向
     * @param: is_z_overlook, 决定z的顺序，true则物体向下倾斜
     */
    void pushPoints3D(double x_length, double y_length, double z_length = 0, bool is_z_overlook = false);
    void clearPoints3D();
    /** 压入二维点
     * 压入的顺序需要和 pushPoints3D() 一致
     */
    void pushPoints2D(const cv::Point2d& point);
    void clearPoints2D();
    /** 默认使用迭代法
     * SOLVEPNP_ITERATIVE速度较慢，并且三维坐标的四个点必须共面
     * SOLVEPNP_P3P和SOLVEPNP_EPNP不要求点共面，P3P适用于四个点的情况，EPNP适用于五个点及以上的情况
     */
    bool solvePnP(bool useExtrinsicGuess = false, int flags = cv::SOLVEPNP_ITERATIVE);
    //获取3维世界坐标
    const cv::Point3d& getPos3D();
    const cv::Mat& getTvec();
    const cv::Mat& getRvec();
    //debug用于输出所有的参数
    void showParams();
private:
    //相机参数矩阵
    cv::Mat camera_matrix;
    //相机畸变矩阵
    cv::Mat distortion_coef;
    //世界坐标系
    std::vector<cv::Point3d> Points3D;
    //相机坐标系二维点
    std::vector<cv::Point2d> Points2D;
    cv::Mat tvec;
    cv::Mat rvec;
    cv::Point3d tvec_;
};
};


#endif