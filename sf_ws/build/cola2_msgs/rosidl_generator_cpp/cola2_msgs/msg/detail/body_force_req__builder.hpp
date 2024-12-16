// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from cola2_msgs:msg/BodyForceReq.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__BODY_FORCE_REQ__BUILDER_HPP_
#define COLA2_MSGS__MSG__DETAIL__BODY_FORCE_REQ__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "cola2_msgs/msg/detail/body_force_req__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace cola2_msgs
{

namespace msg
{

namespace builder
{

class Init_BodyForceReq_disable_axis
{
public:
  explicit Init_BodyForceReq_disable_axis(::cola2_msgs::msg::BodyForceReq & msg)
  : msg_(msg)
  {}
  ::cola2_msgs::msg::BodyForceReq disable_axis(::cola2_msgs::msg::BodyForceReq::_disable_axis_type arg)
  {
    msg_.disable_axis = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cola2_msgs::msg::BodyForceReq msg_;
};

class Init_BodyForceReq_wrench
{
public:
  explicit Init_BodyForceReq_wrench(::cola2_msgs::msg::BodyForceReq & msg)
  : msg_(msg)
  {}
  Init_BodyForceReq_disable_axis wrench(::cola2_msgs::msg::BodyForceReq::_wrench_type arg)
  {
    msg_.wrench = std::move(arg);
    return Init_BodyForceReq_disable_axis(msg_);
  }

private:
  ::cola2_msgs::msg::BodyForceReq msg_;
};

class Init_BodyForceReq_goal
{
public:
  explicit Init_BodyForceReq_goal(::cola2_msgs::msg::BodyForceReq & msg)
  : msg_(msg)
  {}
  Init_BodyForceReq_wrench goal(::cola2_msgs::msg::BodyForceReq::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return Init_BodyForceReq_wrench(msg_);
  }

private:
  ::cola2_msgs::msg::BodyForceReq msg_;
};

class Init_BodyForceReq_header
{
public:
  Init_BodyForceReq_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BodyForceReq_goal header(::cola2_msgs::msg::BodyForceReq::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_BodyForceReq_goal(msg_);
  }

private:
  ::cola2_msgs::msg::BodyForceReq msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::cola2_msgs::msg::BodyForceReq>()
{
  return cola2_msgs::msg::builder::Init_BodyForceReq_header();
}

}  // namespace cola2_msgs

#endif  // COLA2_MSGS__MSG__DETAIL__BODY_FORCE_REQ__BUILDER_HPP_
