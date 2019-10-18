#ifndef LIBBT_COMPOSITE_NODE_H
#define LIBBT_COMPOSITE_NODE_H

#include "behavior_node.h"

namespace bt{

class CompositeNode: public BehaviorNode{
public:
    CompositeNode(std::string name, BehaviorType behavior_type, const Blackboard::Ptr &blackboard_ptr):
      BehaviorNode::BehaviorNode(name, behavior_type, blackboard_ptr),
      children_node_index_(0) {}

    virtual ~CompositeNode() = default;

    virtual void addChildren(const BehaviorNode::Ptr &child_node_ptr);

    virtual void addChildren(std::initializer_list<BehaviorNode::Ptr> children_node_ptr_list);

    std::vector<BehaviorNode::Ptr> getChildren();

    unsigned int getChildrenIndex();

    unsigned int getChildrenNum();

protected:
    virtual void onInitialize() = 0;
    virtual bt::BehaviorState update() = 0;
    virtual void onTerminate(BehaviorState state) = 0;

    std::vector<BehaviorNode::Ptr> children_node_ptr_;
    unsigned int children_node_index_;

}; //class
}; //namespace bt

#endif //LIBBT_COMPOSITE_NODE_H