// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from cola2_msgs:msg/RPY.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__RPY__BUILDER_HPP_
#define COLA2_MSGS__MSG__DETAIL__RPY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "cola2_msgs/msg/detail/rpy__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace cola2_msgs
{

namespace msg
{

namespace builder
{

class Init_RPY_yaw
{
public:
  explicit Init_RPY_yaw(::cola2_msgs::msg::RPY & msg)
  : msg_(msg)
  {}
  ::cola2_msgs::msg::RPY yaw(::cola2_msgs::msg::RPY::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cola2_msgs::msg::RPY msg_;
};

class Init_RPY_pitch
{
public:
  explicit Init_RPY_pitch(::cola2_msgs::msg::RPY & msg)
  : msg_(msg)
  {}
  Init_RPY_yaw pitch(::cola2_msgs::msg::RPY::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_RPY_yaw(msg_);
  }

private:
  ::cola2_msgs::msg::RPY msg_;
};

class Init_RPY_roll
{
public:
  Init_RPY_roll()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RPY_pitch roll(::cola2_msgs::msg::RPY::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_RPY_pitch(msg_);
  }

private:
  ::cola2_msgs::msg::RPY msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::cola2_msgs::msg::RPY>()
{
  return cola2_msgs::msg::builder::Init_RPY_roll();
}

}  // namespace cola2_msgs

#endif  // COLA2_MSGS__MSG__DETAIL__RPY__BUILDER_HPP_
