#ifndef BT_RETRY_NODE_H
#define BT_RETRY_NODE_H

#include "decorator_node.h"

namespace bt{

class RetryNode: public DecoratorNode{
public:
    RetryNode(std::string name, const Blackboard::Ptr &blackboard_ptr,
            const BehaviorNode::Ptr &child_node_ptr = nullptr,
            int retry_thresh_ = 1):
        DecoratorNode::DecoratorNode(name, BehaviorType::RETRY, blackboard_ptr, child_node_ptr),
        retry_thresh(retry_thresh_),
        retry_num(0){}

    virtual ~RetryNode() = default;

protected:
    virtual void onInitialize();
    virtual BehaviorState update();
    virtual void onTerminate(BehaviorState state);

private:
    //尝试的阈值次数
    int retry_thresh;

    //尝试次数
    int retry_num;

}; //class
}; //namespace

#endif //BT_RETRY_NODE