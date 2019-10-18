#ifndef LIBBT_PARALLEL_NODE_H
#define LIBBT_PARALLEL_NODE_H

#include "composite_node.h"

namespace bt{

class ParallelNode: public CompositeNode{
public:
    ParallelNode(std::string name, const Blackboard::Ptr &blackboard_ptr, unsigned int threshold):
        CompositeNode::CompositeNode(name, BehaviorType::PARALLEL, blackboard_ptr),
        threshold_(threshold),
        success_count_(0),
        failure_count_(0){}

    virtual ~ParallelNode() = default;

protected:
    virtual BehaviorState update();
    virtual void onInitialize();
    virtual void onTerminate(BehaviorState state);

    std::vector<bool> children_node_done_;
    unsigned int success_count_;
    unsigned int failure_count_;
    unsigned int threshold_;
}; //class
}; //namespace bt

#endif //LIBBT_PARALLEL_NODE_H