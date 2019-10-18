#ifndef LIBBT_DECORATOR_NODE_H
#define LIBBT_DECORATOR_NODE_H

#include "behavior_node.h"

namespace bt{

class DecoratorNode: public BehaviorNode{
public:
    DecoratorNode(std::string name, BehaviorType behavior_type, const Blackboard::Ptr &blackboard_ptr,
                   const BehaviorNode::Ptr &child_node_ptr = nullptr):
        BehaviorNode::BehaviorNode(name, behavior_type, blackboard_ptr),
        child_node_ptr_(child_node_ptr){  }

    virtual ~DecoratorNode() = default;

    void setChild(const BehaviorNode::Ptr &child_node_ptr) {
        child_node_ptr_ = child_node_ptr;
        child_node_ptr->setParent(shared_from_this());
    }

protected:
    virtual void onInitialize() = 0;
    virtual BehaviorState update() = 0;
    virtual void onTerminate(BehaviorState state) = 0;
    BehaviorNode::Ptr child_node_ptr_;
}; //class
}; //namespace bt

#endif //LIBBT_DECORATOR_NODE_H