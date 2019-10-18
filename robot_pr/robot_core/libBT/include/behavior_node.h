#ifndef LIBBT_BEHAVIOR_NODE_H
#define LIBBT_BEHAVIOR_NODE_H

#include <chrono>
#include <thread>
#include <vector>
#include <iostream>

#include "blackboard.h"
#include "robot_tool/log_tool.h"

namespace bt{

enum class BehaviorType {
    PARALLEL,
    SELECTOR,
    SEQUENCE,

    ACTION,

    PRECONDITION,
    RETRY,
    TIMEOUT,
    INVERTER,
    FORCE_SUCCESS,
    ENSURE_ENTER
};

enum class BehaviorState {
    RUNNING,
    SUCCESS,
    FAILURE,
    IDLE,
};

enum class AbortType {
    NONE, //无优先级
    SELF, //打断自己的直接子节点
    LOW_PRIORITY, //打断优先级低的节点
    BOTH
};

//所有节点的父类
//enable_shared_from_this的作用:
//  如果使用智能指针，则必须全部使用智能指针，否则会出现一些引用计数的错误
//  不可以直接使用shared_ptr<this>，否则会导致两个shared_ptr对象持有同一个引用，并未引用计数且导致两次析构报错
class BehaviorNode : public std::enable_shared_from_this<BehaviorNode>{
public:
    typedef std::shared_ptr<BehaviorNode> Ptr;

    BehaviorNode(std::string name, BehaviorType behavior_type, const Blackboard::Ptr &blackboard_ptr):
        name_(name),
        behavior_type_(behavior_type),
        blackboard_ptr_(blackboard_ptr),
        behavior_state_(BehaviorState::IDLE) {}
    
    virtual ~BehaviorNode() = default;

    virtual void reset();

    BehaviorState run();

    BehaviorType getBehaviorType();

    BehaviorState getBehaviorState();

    std::string getName();

    void setParent(BehaviorNode::Ptr parent_node_ptr);

 protected:
    virtual BehaviorState update() = 0;
    virtual void onInitialize() = 0;
    virtual void onTerminate(BehaviorState state) = 0;

    //! Node name
    std::string name_;
    //! State
    //std::mutex behavior_state_mutex_;
    BehaviorState behavior_state_;
    //! Type
    BehaviorType behavior_type_;
    //! Blackboard
    Blackboard::Ptr blackboard_ptr_;
    //! Parent Node Pointer
    BehaviorNode::Ptr parent_node_ptr_;
}; //class
}; //namespace bt

#endif //LIBBT_BEHAVIOR_NODE_H