#include "timeout_node.h"

using namespace sentry_tool_uestc;

void bt::TimeoutNode::onInitialize(){
    LOG_INFO<<name_<<" "<<__FUNCTION__;
}

bt::BehaviorState bt::TimeoutNode::update(){
    if(child_node_ptr_ == nullptr){
        return BehaviorState::SUCCESS;
    }
    BehaviorState state = child_node_ptr_->run();
    if(state == BehaviorState::RUNNING){
        if(getBehaviorState() != BehaviorState::RUNNING){
            start_time = robot_tool::TimeTool::getCurrTime();
        }
        if(robot_tool::TimeTool::countTimeDuration(start_time) > out_duration){
            if(timeout_function){
                timeout_function();
            }
            return BehaviorState::FAILURE;
        }
    }
    return state;
}

void bt::TimeoutNode::onTerminate(BehaviorState state){
    switch (state){
        case BehaviorState::IDLE:
            LOG_INFO<<name_<<" "<<__FUNCTION__<<" IDLE!";
            child_node_ptr_->reset();
            break;
        case BehaviorState::SUCCESS:
            LOG_INFO<<name_<<" "<<__FUNCTION__<<" SUCCESS!";
            break;
        case BehaviorState::FAILURE:
            LOG_INFO<<name_<<" "<<__FUNCTION__<<" FAILURE!";
            child_node_ptr_->reset();
            break;
        default:
            LOG_ERROR<<name_<<" "<<__FUNCTION__<<" ERROR!";
            return;
    }
}