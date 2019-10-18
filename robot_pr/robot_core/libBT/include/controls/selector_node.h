#ifndef LIBBT_SELECTOR_NODE_H
#define LIBBT_SELECTOR_NODE_H

#include "composite_node.h"
#include "decorators/precondition_node.h"

namespace bt{

class SelectorNode: public CompositeNode{
public:
    SelectorNode(std::string name, const Blackboard::Ptr &blackboard_ptr):
        CompositeNode::CompositeNode(name, BehaviorType::SELECTOR, blackboard_ptr) {}

    virtual ~SelectorNode() = default;

    virtual void addChildren(const BehaviorNode::Ptr &child_node_ptr);

    virtual void addChildren(std::initializer_list<BehaviorNode::Ptr> children_node_ptr_list);

    void setChildrenIndex(unsigned int children_node_index);

protected:
    virtual void onInitialize();
    virtual bt::BehaviorState update();
    virtual void onTerminate(BehaviorState state);

    std::vector<bool> children_node_reevaluation_;

}; //class
}; //namespace bt

#endif //LIBBT_SELECTOR_NODE_H