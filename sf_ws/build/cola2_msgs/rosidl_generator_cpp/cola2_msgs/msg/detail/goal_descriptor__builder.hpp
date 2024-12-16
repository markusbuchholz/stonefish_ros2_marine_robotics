// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from cola2_msgs:msg/GoalDescriptor.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__GOAL_DESCRIPTOR__BUILDER_HPP_
#define COLA2_MSGS__MSG__DETAIL__GOAL_DESCRIPTOR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "cola2_msgs/msg/detail/goal_descriptor__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace cola2_msgs
{

namespace msg
{

namespace builder
{

class Init_GoalDescriptor_priority
{
public:
  explicit Init_GoalDescriptor_priority(::cola2_msgs::msg::GoalDescriptor & msg)
  : msg_(msg)
  {}
  ::cola2_msgs::msg::GoalDescriptor priority(::cola2_msgs::msg::GoalDescriptor::_priority_type arg)
  {
    msg_.priority = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cola2_msgs::msg::GoalDescriptor msg_;
};

class Init_GoalDescriptor_requester
{
public:
  Init_GoalDescriptor_requester()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GoalDescriptor_priority requester(::cola2_msgs::msg::GoalDescriptor::_requester_type arg)
  {
    msg_.requester = std::move(arg);
    return Init_GoalDescriptor_priority(msg_);
  }

private:
  ::cola2_msgs::msg::GoalDescriptor msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::cola2_msgs::msg::GoalDescriptor>()
{
  return cola2_msgs::msg::builder::Init_GoalDescriptor_requester();
}

}  // namespace cola2_msgs

#endif  // COLA2_MSGS__MSG__DETAIL__GOAL_DESCRIPTOR__BUILDER_HPP_
