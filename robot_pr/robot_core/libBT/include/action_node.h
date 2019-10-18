#ifndef LIBBT_ACTION_NODE_H
#define LIBBT_ACTION_NODE_H

#include "behavior_node.h"

namespace bt{

class ActionNode: public BehaviorNode{
public:
    ActionNode(std::string name, const Blackboard::Ptr &blackboard_ptr):
        bt::BehaviorNode::BehaviorNode(name, BehaviorType::ACTION, blackboard_ptr){}

    virtual ~ActionNode() = default;

protected:
    virtual void onInitialize() = 0;
    virtual bt::BehaviorState update() = 0;
    virtual void onTerminate(BehaviorState state) = 0;

}; //class
}; //namespace bt

#endif //LIBBT_ACTION_NODE_H