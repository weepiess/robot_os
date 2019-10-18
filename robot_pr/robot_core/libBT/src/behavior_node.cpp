#include "behavior_node.h"

bt::BehaviorState bt::BehaviorNode::run(){
    if (behavior_state_ != BehaviorState::RUNNING) {
        onInitialize();
    }
    behavior_state_ = update();
    if (behavior_state_ != BehaviorState::RUNNING) {
        onTerminate(behavior_state_);
    }
    return behavior_state_;
}

void bt::BehaviorNode::reset(){
    if (behavior_state_ == BehaviorState::RUNNING){
        behavior_state_ = BehaviorState::IDLE;
        onTerminate(behavior_state_);
    }
}

bt::BehaviorType bt::BehaviorNode::getBehaviorType(){
    return behavior_type_;
}

bt::BehaviorState bt::BehaviorNode::getBehaviorState(){
    return behavior_state_;
}

std::string bt::BehaviorNode::getName(){
    return name_;
}

void bt::BehaviorNode::setParent(BehaviorNode::Ptr parent_node_ptr){
    parent_node_ptr_ = parent_node_ptr;
}