#ifndef BT_ENSURE_ENTER_NODE_H
#define BT_ENSURE_ENTER_NODE_H

#include "decorator_node.h"

namespace bt{

class EnsureEnterNode: public DecoratorNode{
public:
    EnsureEnterNode(std::string name, const Blackboard::Ptr &blackboard_ptr,
            const BehaviorNode::Ptr &child_node_ptr = nullptr,
            int ensure_enter_counts_ = 0):
            ensure_enter_counts(ensure_enter_counts_),
            DecoratorNode::DecoratorNode(name, BehaviorType::ENSURE_ENTER, blackboard_ptr, child_node_ptr){}

    virtual ~EnsureEnterNode() = default;

protected:
    virtual void onInitialize();
    virtual BehaviorState update();
    virtual void onTerminate(BehaviorState state);

private:
    int ensure_enter_counts;

    int curr_ensure_counts = 0;

}; //class
}; //namespace

#endif