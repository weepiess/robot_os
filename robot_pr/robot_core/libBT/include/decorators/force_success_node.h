#ifndef SENTRY_FORCE_SUCCESS_NODE_H
#define SENTRY_FORCE_SUCCESS_NODE_H

#include "decorator_node.h"

namespace bt{

class ForceSuccessNode: public DecoratorNode{
public:
    ForceSuccessNode(std::string name, const Blackboard::Ptr &blackboard_ptr,
            const BehaviorNode::Ptr &child_node_ptr = nullptr):
            DecoratorNode::DecoratorNode(name, BehaviorType::FORCE_SUCCESS, blackboard_ptr, child_node_ptr){}

    virtual ~ForceSuccessNode() = default;

protected:
    virtual void onInitialize();
    virtual BehaviorState update();
    virtual void onTerminate(BehaviorState state);

}; //class
}; //namespace

#endif