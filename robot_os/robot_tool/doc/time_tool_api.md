# time_tool API

## 1.API使用

### 命名空间：

```c++
using namespace robot_tool;
```
### Class

```c++
Class TimeTool
```

- 均是静态方法，可直接使用

### API方法：

__获取当前时间__

```c++
static std::chrono::steady_clock::time_point getCurrTime();
```

__时间段计时__

```c++
// @param: pre_time, 开始计时的时间
// @param: time_units, 计时单位，默认ms
// @return: int64_t，从pre_time开始经历的时间段
static int64_t countTimeDuration(const std::chrono::steady_clock::time_point& pre_time, int time_units = MILLISECONDS);
```

- 和获取当前时间函数配合使用

例子:

```c++
auto start=TimeTool::getCurrTime();
//your code
int i=10000;
while(i--);
//your code
int time_duration=TimeTool::countTimeDuration(start);
std::cout<<"duration:"<<time_duration<<" ms"<<std::endl;
```

__时间段计时（重载）__

```c++
// 重载
// @param: former_time, 计时的开始时间
// @param: later_time, 计时的结束时间
// @param: time_units, 计时单位，默认ms
// @return: int64_t，经历的时间段
static int64_t countTimeDuration(const std::chrono::steady_clock::time_point& former_time, 
            const std::chrono::steady_clock::time_point& later_time, int time_units = MILLISECONDS);
```

例子：

```c++
auto start=TimeTool::getCurrTime();
//your code
int i=10000;
while(i--);
//your code
auto end=TimeTool::getCurrTime();

int time_duration=TimeTool::countTimeDuration(start,end);
std::cout<<"duration:"<<time_duration<<" ms"<<std::endl;
```

## 2.使用技巧

相关使用技巧：

- 使用auto关键字表示std::chrono::steady_clock::time_point类型，可以使代码简洁

用处：

- 可以测试代码段的运行时间