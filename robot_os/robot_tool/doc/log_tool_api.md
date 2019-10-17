# log_tool API

## 1.API使用

### 命名空间:

```c++
using namespace robot_tool;
```

### API方法:

__a.日志输出__

```c++
LOG_INFO
LOG_WARNING
LOG_ERROR
LOG_FATAL
```

例子：

```c++
#include "robot_tool/log_tool.h"
LOG_INFO<<"init ok!"<<std::endl;
```

__b.条件输出__

```c++
LOG_INFO_IF(condition)
LOG_WARNING_IF(condition)
LOG_ERROR_IF(condition)
LOG_FATAL_IF(condition)
```

## 2.使用技巧

* 无