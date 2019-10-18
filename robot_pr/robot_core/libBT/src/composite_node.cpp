#include "composite_node.h"

void bt::CompositeNode::addChildren(const BehaviorNode::Ptr &child_node_ptr){
    children_node_ptr_.push_back(child_node_ptr);
    child_node_ptr->setParent(shared_from_this());
}

void bt::CompositeNode::addChildren(std::initializer_list<BehaviorNode::Ptr> child_node_ptr_list){
    for(auto child_node_ptr=child_node_ptr_list.begin(); child_node_ptr!=child_node_ptr_list.end(); child_node_ptr++){
        children_node_ptr_.push_back(*(child_node_ptr));
        (*child_node_ptr)->setParent(shared_from_this());
    }
}

std::vector<bt::BehaviorNode::Ptr> bt::CompositeNode::getChildren(){
    return children_node_ptr_;
}

unsigned int bt::CompositeNode::getChildrenIndex(){
    return children_node_index_;
}

unsigned int bt::CompositeNode::getChildrenNum(){
    return children_node_ptr_.size();
}