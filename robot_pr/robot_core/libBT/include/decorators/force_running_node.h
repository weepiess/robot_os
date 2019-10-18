#ifndef BT_FORCE_RUNNING_NODE_H
#define BT_FORCE_RUNNING_NODE_H

#include "decorator_node.h"

namespace bt{

class ForceRunningNode: public DecoratorNode{
public:
    ForceRunningNode(std::string name, const Blackboard::Ptr &blackboard_ptr,
            const BehaviorNode::Ptr &child_node_ptr = nullptr):
            DecoratorNode::DecoratorNode(name, BehaviorType::FORCE_SUCCESS, blackboard_ptr, child_node_ptr){}

    virtual ~ForceRunningNode() = default;

protected:
    virtual void onInitialize();
    virtual BehaviorState update();
    virtual void onTerminate(BehaviorState state);

}; //class
}; //namespace

#endif