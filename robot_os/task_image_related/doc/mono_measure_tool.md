# MonoMeasureTool模块使用说明

该模块包含部分单目视觉的算法封装，需要相机的内参矩阵和畸变参数。

__参数配置文件:__　yaml格式

```yaml
intrinsic_matrix: !!opencv-matrix
  rows: 3
  cols: 3
  dt: d
  data: [1633.310586,0.000000,632.918617, 0.000000,1635.296201,537.038591, 0.000000, 0.000000, 1.000000]
distortion_coeffs: !!opencv-matrix
  rows: 1
  cols: 5
  dt: d
  data: [-0.094445,0.253880,0.005805 ,-0.000319,0.000000]
```

__初始化__

```c++
MonoMeasureTool tool;
tool.init(config_path);//需要传入配置文件路径
```

#### a.3d点求解

__利用pnp算法：__

```c++
//////////3d点坐标求解（use solve pnp）
// points2d: input,一组图像上的2d点（4个点）
// points3d: input,一组3d点（世界坐标系），对应图像上的点（4个点）
// position: output,世界坐标系原点在相机坐标系下的位置。
// return :state
int solvePnP4Points(std::vector<cv::Point2f>& points2d,
                        std::vector<cv::Point3f>& points3d,
                        cv::Point3f& position);
```

* 内部使用的是opencv中的solvePnP函数
* 需要至少4个点。（points2d的点与points3d的点要一一对应）
  * 参数points2d，使用图像处理的方法，得到图像上对应的四个点，
  * 参数points3d是一个固定的参数，以__想要求解的点__为原点建立世界坐标系(不要求坐标系方向)，得到对应四个点的坐标，作为该参数。

__相似三角形投影算法：__

```c++
//////3d点坐标求解
//p: intput,图像上点坐标
//distance: input,已知的真实距离
//return :对应的真实3d点坐标 
cv::Point3f imagePoint2CameraFrame(cv::Point2f p, double distance);
```

* 参考http://www.cnblogs.com/singlex/p/pose_estimation_1_1.html

#### b.视角求解

```c++
//////视角求解
//p: intput,图像上点坐标
//pitch: output,视角pitch
//yaw: output,视角yaw
//return :state 
int imagePoint2ViewAngle(cv::Point2f p, float& pitch, float& yaw);
```

