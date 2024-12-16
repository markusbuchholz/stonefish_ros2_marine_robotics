// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from cola2_msgs:msg/Setpoints.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__SETPOINTS__BUILDER_HPP_
#define COLA2_MSGS__MSG__DETAIL__SETPOINTS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "cola2_msgs/msg/detail/setpoints__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace cola2_msgs
{

namespace msg
{

namespace builder
{

class Init_Setpoints_setpoints
{
public:
  explicit Init_Setpoints_setpoints(::cola2_msgs::msg::Setpoints & msg)
  : msg_(msg)
  {}
  ::cola2_msgs::msg::Setpoints setpoints(::cola2_msgs::msg::Setpoints::_setpoints_type arg)
  {
    msg_.setpoints = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cola2_msgs::msg::Setpoints msg_;
};

class Init_Setpoints_header
{
public:
  Init_Setpoints_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Setpoints_setpoints header(::cola2_msgs::msg::Setpoints::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Setpoints_setpoints(msg_);
  }

private:
  ::cola2_msgs::msg::Setpoints msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::cola2_msgs::msg::Setpoints>()
{
  return cola2_msgs::msg::builder::Init_Setpoints_header();
}

}  // namespace cola2_msgs

#endif  // COLA2_MSGS__MSG__DETAIL__SETPOINTS__BUILDER_HPP_
