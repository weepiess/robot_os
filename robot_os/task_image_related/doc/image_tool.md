# ImageTool模块使用文档

主要包括两部分，图像调试和图像数学计算。

* 该类为静态类，其函数可以直接使用。

#### a.图像调试

静态调试开关：控制是否显示调试信息

```c++
//默认为false，即调用该类的调试函数，均不会执行。
task_image_related::ImageTool::is_debug=true
```

调试函数（受静态调试开关控制）

```c++
//宏定义IMAGE_TOOL_DEBUG();执行括号中的语句
IMAGE_TOOL_DEBUG(imshow("dst", dst));
//api　type:0,blue;1,green;2,red.
static void drawRotatedRect(cv::Mat &img,cv::RotatedRect r,int type=0);
static void draw4Point4f(cv::Mat &img, cv::Point2f *point2fs,int type=0);
static void drawConvexHull(cv::Mat &img,std::vector<cv::Point2f> points,int type=0);
//usage example,draw a RotatedRect
task_image_related::ImageTool::drawRotatedRect(img,r)//default blue line
```

#### b.图像数学计算

```c++
//两点间的距离
static float calc2PointDistance(cv::Point2f point1,cv::Point2f point2);
//两点间的距离（重载，3d点）
static float calc2PointDistance(cv::Point3f point1,cv::Point3f point2);
//两点构成直线的倾角，相对常规坐标系，图像坐标系和常规坐标系不统一
static float calc2PointAngle(cv::Point2f point1,cv::Point2f point2);
//三角形的角度，第一个参数为顶点坐标
static float calcTriangleInnerAngle(cv::Point2f vertexPoint,cv::Point2f point1,cv::Point2f point2);
//usage example
ImageTool::calc2PointDistance(point1, point2);//计算两点距离
```

