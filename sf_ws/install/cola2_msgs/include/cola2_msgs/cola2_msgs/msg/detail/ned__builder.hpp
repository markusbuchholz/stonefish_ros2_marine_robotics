// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from cola2_msgs:msg/NED.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__NED__BUILDER_HPP_
#define COLA2_MSGS__MSG__DETAIL__NED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "cola2_msgs/msg/detail/ned__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace cola2_msgs
{

namespace msg
{

namespace builder
{

class Init_NED_depth
{
public:
  explicit Init_NED_depth(::cola2_msgs::msg::NED & msg)
  : msg_(msg)
  {}
  ::cola2_msgs::msg::NED depth(::cola2_msgs::msg::NED::_depth_type arg)
  {
    msg_.depth = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cola2_msgs::msg::NED msg_;
};

class Init_NED_east
{
public:
  explicit Init_NED_east(::cola2_msgs::msg::NED & msg)
  : msg_(msg)
  {}
  Init_NED_depth east(::cola2_msgs::msg::NED::_east_type arg)
  {
    msg_.east = std::move(arg);
    return Init_NED_depth(msg_);
  }

private:
  ::cola2_msgs::msg::NED msg_;
};

class Init_NED_north
{
public:
  Init_NED_north()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NED_east north(::cola2_msgs::msg::NED::_north_type arg)
  {
    msg_.north = std::move(arg);
    return Init_NED_east(msg_);
  }

private:
  ::cola2_msgs::msg::NED msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::cola2_msgs::msg::NED>()
{
  return cola2_msgs::msg::builder::Init_NED_north();
}

}  // namespace cola2_msgs

#endif  // COLA2_MSGS__MSG__DETAIL__NED__BUILDER_HPP_
