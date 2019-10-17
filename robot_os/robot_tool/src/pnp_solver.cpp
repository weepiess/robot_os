#include "robot_tool/pnp_solver.h"

using namespace std;
using namespace cv;
using namespace robot_tool;

PNPSolver::PNPSolver(){}
PNPSolver::~PNPSolver(){}

void PNPSolver::setCameraMatrix(float fx, float fy, float cx, float cy){
    camera_matrix = (Mat_<float>(3,3) << fx,0,cx,0,fy,cy,0,0,1);
}

void PNPSolver::setDistortionCoef(float rdx, float rdy, float tdx, float tdy){
    distortion_coef = (Mat_<float>(4,1) << rdx,rdy,tdx,tdy);
}

void PNPSolver::pushPoints3D(const Point3d& point){
    if(Points3D.size() > 4){
        cout<<"you have selected over four points, check your method in PNPSolver::solvePnP() to ensure your input."<<endl
            <<"current Points3D.size() is: "<<Points3D.size()<<endl;
    }
    Points3D.emplace_back(point);
}

void PNPSolver::pushPoints3D(double x_length, double y_length, double z_length, bool is_z_overlook){
    if(Points3D.size() > 4){
        cout<<"you have selected over four points, check your method in PNPSolver::solvePnP() to ensure your input."<<endl
            <<"current Points3D.size() is: "<<Points3D.size()<<endl;
    }
    //是否俯视
    if(is_z_overlook){
        z_length = -z_length;
    }
    //默认顺时针
    Points3D.emplace_back(Point3d(-x_length/2, -y_length/2, -z_length/2));
    Points3D.emplace_back(Point3d(x_length/2, -y_length/2, -z_length/2));
    Points3D.emplace_back(Point3d(x_length/2, y_length/2, z_length/2));
    Points3D.emplace_back(Point3d(-x_length/2, y_length/2, z_length/2));
}

void PNPSolver::clearPoints3D(){
    Points3D.clear();
}

void PNPSolver::pushPoints2D(const Point2d& points){
    if(Points3D.size()!=0 && Points2D.size()>Points3D.size()){
        cout<<"Points2D.size() is not equal to Points3D.size(), can not push more points."<<endl
            <<"Points2D.size() is: "<<Points2D.size()<<" "<<"Points3D.size() is: "<<Points3D.size()<<endl;
        return;
    }
    Points2D.emplace_back(points);
}

void PNPSolver::clearPoints2D(){
    Points2D.clear();
}

bool PNPSolver::solvePnP(bool useExtrinsicGuess, int flags){
    if(camera_matrix.empty() && distortion_coef.empty())
        return false;
    if(Points3D.size()==0){
        cout<<"Points3D has not initialized."<<endl;
        return false;
    }
    if(Points3D.size() != Points2D.size()){
        cout<<"Points.size() match error."<<endl
            <<"Points2D.size() is: "<<Points2D.size()<<" "<<"Points3D.size() is: "<<Points3D.size()<<endl;
        return false;
    }
    return cv::solvePnP(Points3D, Points2D, camera_matrix, distortion_coef, rvec, tvec, useExtrinsicGuess, flags);
}

const Point3d& PNPSolver::getPos3D(){
    if(tvec.empty()){
        tvec_.x = tvec_.y = tvec_.z = 0;
    } else {
        //tvec返回的是double类型
        tvec_.x = tvec.at<double>(0);
        tvec_.y = tvec.at<double>(1);
        tvec_.z = tvec.at<double>(2);
    }
    return tvec_;
}

const cv::Mat& PNPSolver::getTvec(){
    return tvec;
}

const cv::Mat& PNPSolver::getRvec(){
    return rvec;
}

void PNPSolver::showParams(){
    cout<<"-----------------------------"<<endl;
    cout<<"PNPSovler's params:"<<endl;
    cout<<"camera_matrix:   "<<camera_matrix<<endl;
    cout<<"distortion_coef:   "<<distortion_coef<<"\n"<<endl;
    cout<<"Points3D:\n"<<Points3D<<"\n"<<endl;
    cout<<"Points2D:\n"<<Points2D<<endl;
    cout<<"-----------------------------"<<endl;
}
