#include "selector_node.h"

void bt::SelectorNode::addChildren(const BehaviorNode::Ptr &child_node_ptr){
    bt::CompositeNode::addChildren(child_node_ptr);

    children_node_reevaluation_.push_back
        (child_node_ptr->getBehaviorType()==BehaviorType::PRECONDITION
             && (std::dynamic_pointer_cast<bt::PreconditionNode>(child_node_ptr)->getAbortType() == AbortType::LOW_PRIORITY
                ||std::dynamic_pointer_cast<bt::PreconditionNode>(child_node_ptr)->getAbortType() == AbortType::BOTH));
}

void bt::SelectorNode::addChildren(std::initializer_list<BehaviorNode::Ptr> children_node_ptr_list){
    bt::CompositeNode::addChildren(children_node_ptr_list);
    
    for(auto child_node_ptr=children_node_ptr_list.begin(); child_node_ptr!=children_node_ptr_list.end(); child_node_ptr++){
        children_node_reevaluation_.push_back
            ((*child_node_ptr)->getBehaviorType()==BehaviorType::PRECONDITION
                && (std::dynamic_pointer_cast<bt::PreconditionNode>(*child_node_ptr)->getAbortType() == AbortType::LOW_PRIORITY
                    ||std::dynamic_pointer_cast<bt::PreconditionNode>(*child_node_ptr)->getAbortType() == AbortType::BOTH));
    }
}

void bt::SelectorNode::setChildrenIndex(unsigned int children_node_index){
    children_node_index_ = children_node_index;
}

bt::BehaviorState bt::SelectorNode::update(){
    if (children_node_ptr_.size() == 0) {
        return BehaviorState::SUCCESS;
    }

    //Reevaluation
    for(unsigned int index = 0; index < children_node_index_; index++){
        LOG_INFO << "Reevaluation";
        if (children_node_reevaluation_.at(index)){
            //这步可能会将children_node_index_修改为index，也就是改为运行index代表的节点
            BehaviorState state = children_node_ptr_.at(index)->run();
            if(index == children_node_index_){
                LOG_INFO<<name_<<" abort goes on! ";
                if (state != BehaviorState::FAILURE) {
                    return state;
                }
                ++children_node_index_;
                break;
            }
        }
    }

    while(true){
        BehaviorState state = children_node_ptr_.at(children_node_index_)->run();

        if (state != BehaviorState::FAILURE) {
            return state;
        }

        if (++children_node_index_ == children_node_ptr_.size()) {
            children_node_index_ = 0;
            return BehaviorState::FAILURE;
        }
    }
}

void bt::SelectorNode::onInitialize(){
    children_node_index_ = 0;
    LOG_INFO<<name_<<" "<<__FUNCTION__;
}

void bt::SelectorNode::onTerminate(BehaviorState state){
    switch (state){
        case BehaviorState::IDLE:
            LOG_INFO<<name_<<" "<<__FUNCTION__<<" IDLE!";
            //TODO: the following recovery measure is called by parent node, and deliver to reset its running child node
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