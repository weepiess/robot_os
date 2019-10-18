////////////////////////////////////////////////////////////////////////////////
///Copyright(c)      UESTC ROBOMASTER2018      Tool Code for robot
///ALL RIGHTS RESERVED
///@file:basic_tool.cpp
///@brief: 无。
///@vesion 1.0
///@author: gezp
///@email: 1350824033@qq.com
///@date: 18-6-13
///修订历史：
////////////////////////////////////////////////////////////////////////////////
#include "robot_tool_pr/basic_tool.h"
#include "robot_tool/log_tool.h"

using namespace sentry_tool_uestc;
using namespace robot_tool;
using namespace std;

void BasicTool::readCameraParams(const string& file_name, PNPSolver& pnp_solver){
    cv::FileStorage cameraFile(file_name, cv::FileStorage::READ);
    if (!cameraFile.isOpened()){
        LOG_ERROR<<"Failed to open "<<file_name;
        return;
    }
    float fx, fy, cx, cy, rdx, rdy, tdx, tdy;
    cameraFile["fx"] >> fx;
    cameraFile["fy"] >> fy;
    cameraFile["cx"] >> cx;
    cameraFile["cy"] >> cy;
    cameraFile["rdx"] >> rdx;
    cameraFile["rdy"] >> rdy;
    cameraFile["tdx"] >> tdx;
    cameraFile["tdy"] >> tdy;
    pnp_solver.setCameraMatrix(fx, fy, cx, cy);
    pnp_solver.setDistortionCoef(rdx, rdy, tdx, tdy);
    cameraFile.release();
}

void BasicTool::readCameraParams(const string& file_name, vector<PNPSolver*>& pnp_solvers){
    cv::FileStorage cameraFile(file_name, cv::FileStorage::READ);
    if (!cameraFile.isOpened()){
        LOG_ERROR<<"Failed to open "<<file_name;
        return;
    }
    float fx, fy, cx, cy, rdx, rdy, tdx, tdy;
    cameraFile["fx"] >> fx;
    cameraFile["fy"] >> fy;
    cameraFile["cx"] >> cx;
    cameraFile["cy"] >> cy;
    cameraFile["rdx"] >> rdx;
    cameraFile["rdy"] >> rdy;
    cameraFile["tdx"] >> tdx;
    cameraFile["tdy"] >> tdy;
    for(int i=0; i<pnp_solvers.size(); i++){
        pnp_solvers[i]->setCameraMatrix(fx, fy, cx, cy);
        pnp_solvers[i]->setDistortionCoef(rdx, rdy, tdx, tdy);
    }
    cameraFile.release();
}

float BasicTool::calTargetYDis(double y, double z, float curr_pitch){
    return z*sin(curr_pitch/180*CV_PI) + y*cos(curr_pitch/180*CV_PI);
}

int BasicTool::getRandomPos(int min_pos, int max_pos){
    int interval = max_pos - min_pos;
    return min_pos + (rand() % (interval + 1));
}