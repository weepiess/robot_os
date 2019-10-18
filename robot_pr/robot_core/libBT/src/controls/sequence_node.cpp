#include "sequence_node.h"

bt::BehaviorState bt::SequenceNode::update(){
    if (children_node_ptr_.size() == 0) {
        return BehaviorState::SUCCESS;
    }

    while(true){
        BehaviorState state = children_node_ptr_.at(children_node_index_)->run();

        if (state != BehaviorState::SUCCESS) {
            return state;
        }

        if (++children_node_index_ == children_node_ptr_.size()) {
            children_node_index_ = 0;
            return BehaviorState::SUCCESS;
        }
    }
}

void bt::SequenceNode::onInitialize(){
    children_node_index_ = 0;
    LOG_INFO<<name_<<" "<<__FUNCTION__;
}

void bt::SequenceNode::onTerminate(BehaviorState state){
    switch (state){
        case BehaviorState::IDLE:
            LOG_INFO<<name_<<" "<<__FUNCTION__<<" IDLE!";
            children_node_ptr_.at(children_node_index_)->reset();
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