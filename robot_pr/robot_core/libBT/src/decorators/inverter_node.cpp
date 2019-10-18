#include "inverter_node.h"

void bt::InverterNode::onInitialize(){
    LOG_INFO<<name_<<" "<<__FUNCTION__;
}

bt::BehaviorState bt::InverterNode::update(){
    if(child_node_ptr_ == nullptr){
        return BehaviorState::SUCCESS;
    }
    BehaviorState state = child_node_ptr_->run();
    switch(state){
        case BehaviorState::SUCCESS:{
            return BehaviorState::FAILURE;
        }
        case BehaviorState::FAILURE:{
            return BehaviorState::SUCCESS;
        }
        case BehaviorState::RUNNING:{
            return BehaviorState::RUNNING;
        }
        default:
            //TODO: throw?
            break;
    }
}

void bt::InverterNode::onTerminate(BehaviorState state){
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