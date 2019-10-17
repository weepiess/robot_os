#ifndef BASE_THREAD
#define BASE_THREAD

#include <thread>
#include <atomic>

namespace robot_tool {

    class BaseThread{
    public:
        BaseThread();
        virtual ~BaseThread();

    public:
        //创建线程并运行
        virtual void start();

        //获取当前线程id
        std::thread::id getId();

        //中断线程
        void interrupt();

        //获取中断信息
        bool isInterrupted();

        //等待线程运行结束
        void join();

    protected:
        //线程运行函数
        virtual void run() = 0;

    private:
        //执行线程
        std::thread execute_thread;

        //中断标志位
        std::atomic<bool> is_interrupt{false};
    };

};

#endif //BASE_THREAD

/*
example:

------------my_thread.h---------------
#include "base_thread.h"

class MyThread: public BaseThread{
public:
	MyThread();
	virtual ~MyThread();
 
private:
	virtual void run() override; //you must override it in .cpp
};

-------------my_thread.cpp--------------
#include "my_thread.h"

void MyThread::run(){
    while(!isInterrupted()){
        //run your code
    }
}

---------------main.cpp-----------------
int main(){
    MyThread my_thread;
    my_thread.start();

    //some code

    my_thread.interrupt(); //if need

    return 0;
}
*/