#include "robot_tool/multi_thread.h"
#include <iostream>

using namespace robot_tool;

BaseThread::BaseThread(){}

BaseThread::~BaseThread(){
    if(!isInterrupted()){
        interrupt();
    }
    join();
}

void BaseThread::start(){
    std::thread temp_thread(std::bind(&BaseThread::run, this));
    execute_thread = std::move(temp_thread);
}

std::thread::id BaseThread::getId(){
    return execute_thread.get_id();
}

void BaseThread::interrupt(){
    is_interrupt = true;
}

bool BaseThread::isInterrupted(){
    return is_interrupt;
}

void BaseThread::join(){
    if(execute_thread.joinable()){
        execute_thread.join();
    }
}