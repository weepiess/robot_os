#include "ensure_enter_node.h"

void bt::EnsureEnterNode::onInitialize(){
    LOG_INFO<<name_<<" "<<__FUNCTION__;
}

bt::BehaviorState bt::EnsureEnterNode::update(){
    if(child_node_ptr_ == nullptr){
        return BehaviorState::SUCCESS;
    }
    if(curr_ensure_counts < ensure_enter_counts){ //未满足进入条件次数
        curr_ensure_counts++;
        return BehaviorState::FAILURE;
    }
    BehaviorState state = child_node_ptr_->run();
    return state;
}

void bt::EnsureEnterNode::onTerminate(BehaviorState state){
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
}