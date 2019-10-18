#ifndef SENTRY_INVERTER_NODE_H
#define SENTRY_INVERTER_NODE_H

#include "decorator_node.h"

namespace bt{

class InverterNode: public DecoratorNode{
public:
    InverterNode(std::string name, const Blackboard::Ptr &blackboard_ptr,
            const BehaviorNode::Ptr &child_node_ptr = nullptr):
            DecoratorNode::DecoratorNode(name, BehaviorType::INVERTER, blackboard_ptr, child_node_ptr){}

    virtual ~InverterNode() = default;

protected:
    virtual void onInitialize();
    virtual BehaviorState update();
    virtual void onTerminate(BehaviorState state);

}; //class
}; //namespace

#endif