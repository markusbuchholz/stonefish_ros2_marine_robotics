// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from cola2_msgs:msg/RecoveryAction.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__RECOVERY_ACTION__BUILDER_HPP_
#define COLA2_MSGS__MSG__DETAIL__RECOVERY_ACTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "cola2_msgs/msg/detail/recovery_action__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace cola2_msgs
{

namespace msg
{

namespace builder
{

class Init_RecoveryAction_error_string
{
public:
  explicit Init_RecoveryAction_error_string(::cola2_msgs::msg::RecoveryAction & msg)
  : msg_(msg)
  {}
  ::cola2_msgs::msg::RecoveryAction error_string(::cola2_msgs::msg::RecoveryAction::_error_string_type arg)
  {
    msg_.error_string = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cola2_msgs::msg::RecoveryAction msg_;
};

class Init_RecoveryAction_error_level
{
public:
  explicit Init_RecoveryAction_error_level(::cola2_msgs::msg::RecoveryAction & msg)
  : msg_(msg)
  {}
  Init_RecoveryAction_error_string error_level(::cola2_msgs::msg::RecoveryAction::_error_level_type arg)
  {
    msg_.error_level = std::move(arg);
    return Init_RecoveryAction_error_string(msg_);
  }

private:
  ::cola2_msgs::msg::RecoveryAction msg_;
};

class Init_RecoveryAction_header
{
public:
  Init_RecoveryAction_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RecoveryAction_error_level header(::cola2_msgs::msg::RecoveryAction::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RecoveryAction_error_level(msg_);
  }

private:
  ::cola2_msgs::msg::RecoveryAction msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::cola2_msgs::msg::RecoveryAction>()
{
  return cola2_msgs::msg::builder::Init_RecoveryAction_header();
}

}  // namespace cola2_msgs

#endif  // COLA2_MSGS__MSG__DETAIL__RECOVERY_ACTION__BUILDER_HPP_
