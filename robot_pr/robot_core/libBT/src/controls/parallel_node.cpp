#include "parallel_node.h"

bt::BehaviorState bt::ParallelNode::update(){
    if (children_node_ptr_.size() == 0) {
        return BehaviorState::SUCCESS;
    }

    for (unsigned int index = 0; index!=children_node_ptr_.size(); index++) {
        if (children_node_done_.at(index) == false){
            BehaviorState state = children_node_ptr_.at(index)->run();

            if (state == BehaviorState::SUCCESS) {
                children_node_done_.at(index) = true;
                if (++success_count_ >= threshold_) {
                    return BehaviorState::SUCCESS;
                }
            } else if (state == BehaviorState::FAILURE) {
                children_node_done_.at(index) = true;
                if (++failure_count_ >= children_node_ptr_.size()-threshold_) {
                    return BehaviorState::FAILURE;
                }
            }
        }
    }
    return BehaviorState::RUNNING;
}

void bt::ParallelNode::onInitialize(){
    failure_count_=0;
    success_count_=0;
    children_node_done_.clear();
    children_node_done_.resize(children_node_ptr_.size(),false);
    LOG_INFO<<name_<<" "<<__FUNCTION__;
}

void bt::ParallelNode::onTerminate(BehaviorState state){
    switch (state){
        case BehaviorState::IDLE:
            LOG_INFO<<name_<<" "<<__FUNCTION__<<" IDLE!";
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
    //TODO: no matter what state, the node would reset all running children to terminate.
    for (unsigned int index = 0; index!=children_node_ptr_.size(); index++) {
        children_node_ptr_.at(index)->reset();
    }
}
