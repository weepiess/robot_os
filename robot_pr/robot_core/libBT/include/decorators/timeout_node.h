#ifndef BT_TIMEOUT_NODE_H
#define BT_TIMEOUT_NODE_H

#include "decorator_node.h"
#include "robot_tool_pr/basic_tool.h"
#include <functional>

namespace bt{

class TimeoutNode: public DecoratorNode{
public:
    TimeoutNode(std::string name, const Blackboard::Ptr &blackboard_ptr,
            int64_t milliseconds_out_duration,
            const BehaviorNode::Ptr &child_node_ptr = nullptr,
            std::function<void()> timeout_function_ = std::function<void()>()):
        DecoratorNode::DecoratorNode(name, BehaviorType::TIMEOUT, blackboard_ptr, child_node_ptr),
        out_duration(milliseconds_out_duration),
        timeout_function(timeout_function_){}

    virtual ~TimeoutNode() noexcept = default;

protected:
    virtual void onInitialize();
    virtual BehaviorState update();
    virtual void onTerminate(BehaviorState state);

private:
    std::chrono::steady_clock::time_point start_time;

    int64_t out_duration;

    std::function<void()> timeout_function;
};
};

#endif