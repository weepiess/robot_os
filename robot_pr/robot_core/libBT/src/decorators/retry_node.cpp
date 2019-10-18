#include "retry_node.h"

void bt::RetryNode::onInitialize(){
    LOG_INFO<<name_<<" "<<__FUNCTION__;
}

bt::BehaviorState bt::RetryNode::update(){
    if(child_node_ptr_ == nullptr){
        return BehaviorState::SUCCESS;
    }
    BehaviorState state = child_node_ptr_->run();
    switch(state){
        case BehaviorState::SUCCESS:{
            return BehaviorState::SUCCESS;
        }
        case BehaviorState::FAILURE:{
            if(++retry_num > retry_thresh){
                return BehaviorState::FAILURE;
            } else {
                return BehaviorState::RUNNING;
            }
        }
        case BehaviorState::RUNNING:{
            return BehaviorState::RUNNING;
        }
        default:
            //TODO: throw?
            break;
    }
}

void bt::RetryNode::onTerminate(BehaviorState state){
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
            break;
        default:
            LOG_ERROR<<name_<<" "<<__FUNCTION__<<" ERROR!";
            return;
    }
    //只要结果不是RUNNING都重置
    retry_num = 0;
}