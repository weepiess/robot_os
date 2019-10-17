# 样例开发说明

该模块实现了一个开发样例，即基于task_image_related的图像实时显示。

实现过程如下:

#### step1.创建具体任务类TaskShowImage

参考task_show_image.h/cpp文件

* 继承TaskImageRelatedBase基类
* 实现相关接口

实现初始化接口

```c++
int TaskShowImage::initTask(std::string conf_path){
    setRunFlag(true);
    cout<<"task show image init"<<endl;
    return 0;
}
```

* 调用父类API，setRunFlag(true)，设置任务初始化后直接开始运行。

该接口为主要接口，必须实现，其中使用imshow函数完成了图像的显示

```c++
void TaskShowImage::taskImageProcess(cv::Mat& img,double img_stamp){
    imshow("show_img",img);
    waitKey(1);
    cout<<"task show image,get image"<<endl;
}
```

可选接口：(测试用，可以不用实现)

```c++
void TaskShowImage::taskImageWait(){
    cout<<"task show image,wait get image"<<endl;
}
void TaskShowImage::taskSleep(){
    cout<<"task show image,sleep......"<<endl;
}
```

#### step2.创建node,启动任务

参考nodes/task_show_image_node.cpp

以下为主要代码解释

```c++
//配置文件路径
std::string filename = ros::package::getPath("task_image_related") + "/res/task_show_image_config.yaml";
//创建任务
task_image_related::TaskShowImage task;
//初始化任务，处理任务开始运行
task.init(filename);
//ros spin 调度线程,这里ROS图片订阅需要spin调度。
ros::AsyncSpinner async_spinner(1);
async_spinner.start();
```

注意：

* 在其他package中使用task_image_related功能，只需要在CMakeLists.txt中添加该包的依赖

```cmake
find_package(catkin REQUIRED COMPONENTS
  ...
  task_image_related
)
```

