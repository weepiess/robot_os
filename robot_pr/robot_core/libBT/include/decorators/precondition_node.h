#ifndef LIBBT_PRECONDITION_NODE_H
#define LIBBT_PRECONDITION_NODE_H

#include "decorator_node.h"
#include "controls/selector_node.h"
#include <algorithm>

namespace bt{

class PreconditionNode: public DecoratorNode{
public:
    PreconditionNode(std::string name, const Blackboard::Ptr &blackboard_ptr,
                    const BehaviorNode::Ptr &child_node_ptr = nullptr,
                    std::function<bool()> precondition_function = std::function<bool()>(),
                    AbortType abort_type = AbortType::NONE):
        DecoratorNode::DecoratorNode(name, BehaviorType::PRECONDITION, blackboard_ptr, child_node_ptr),
        precondition_function_(precondition_function), abort_type_(abort_type){}

    virtual ~PreconditionNode() = default;

    AbortType getAbortType();

protected:
    virtual void onInitialize();
    virtual bool precondition();
    virtual BehaviorState update();
    virtual void onTerminate(BehaviorState state);
    virtual bool reevaluation();

    std::function<bool()> precondition_function_;
    AbortType abort_type_;
}; //class
}; //namespace bt

#endif //LIBBT_PRECONDITION_NODE_H