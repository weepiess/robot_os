#ifndef FUNTION_THREAD
#define FUNTION_THREAD

#include <thread>
#include "robot_tool/multi_thread.h"
#include <functional>

namespace sentry_tool_uestc{

    //用于在特定时候开启线程，运行类中的某个函数
    class FunctionThread: public robot_tool::BaseThread{
    public:
        FunctionThread();
        virtual ~FunctionThread();

    public:
        void init(std::function<void()> function_);

    private:
        virtual void run() override;

        //需要运行的函数
        std::function<void()> function;
    };

};

#endif

/*
example:

---------xxx.h----------
//some variables
FunctionThread function_thread;


---------xxx.cpp----------
in your function{
    function_thread.init(std::bind(&xxx::yyy, this)); // xxx is class name, yyy is function name
    function_thread.start();
}

*/