#include "robot_tool_pr/function_thread.h"
#include <iostream>

using namespace sentry_tool_uestc;

FunctionThread::FunctionThread(){}

FunctionThread::~FunctionThread(){}

void FunctionThread::init(std::function<void()> function_){
    function = std::move(function_);
}

void FunctionThread::run(){
    function();
}