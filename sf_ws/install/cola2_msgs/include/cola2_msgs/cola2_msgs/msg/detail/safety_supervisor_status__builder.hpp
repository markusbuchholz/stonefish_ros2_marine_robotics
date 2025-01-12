// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from cola2_msgs:msg/SafetySupervisorStatus.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__SAFETY_SUPERVISOR_STATUS__BUILDER_HPP_
#define COLA2_MSGS__MSG__DETAIL__SAFETY_SUPERVISOR_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "cola2_msgs/msg/detail/safety_supervisor_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace cola2_msgs
{

namespace msg
{

namespace builder
{

class Init_SafetySupervisorStatus_recovery_action
{
public:
  explicit Init_SafetySupervisorStatus_recovery_action(::cola2_msgs::msg::SafetySupervisorStatus & msg)
  : msg_(msg)
  {}
  ::cola2_msgs::msg::SafetySupervisorStatus recovery_action(::cola2_msgs::msg::SafetySupervisorStatus::_recovery_action_type arg)
  {
    msg_.recovery_action = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cola2_msgs::msg::SafetySupervisorStatus msg_;
};

class Init_SafetySupervisorStatus_status_code
{
public:
  explicit Init_SafetySupervisorStatus_status_code(::cola2_msgs::msg::SafetySupervisorStatus & msg)
  : msg_(msg)
  {}
  Init_SafetySupervisorStatus_recovery_action status_code(::cola2_msgs::msg::SafetySupervisorStatus::_status_code_type arg)
  {
    msg_.status_code = std::move(arg);
    return Init_SafetySupervisorStatus_recovery_action(msg_);
  }

private:
  ::cola2_msgs::msg::SafetySupervisorStatus msg_;
};

class Init_SafetySupervisorStatus_header
{
public:
  Init_SafetySupervisorStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SafetySupervisorStatus_status_code header(::cola2_msgs::msg::SafetySupervisorStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_SafetySupervisorStatus_status_code(msg_);
  }

private:
  ::cola2_msgs::msg::SafetySupervisorStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::cola2_msgs::msg::SafetySupervisorStatus>()
{
  return cola2_msgs::msg::builder::Init_SafetySupervisorStatus_header();
}

}  // namespace cola2_msgs

#endif  // COLA2_MSGS__MSG__DETAIL__SAFETY_SUPERVISOR_STATUS__BUILDER_HPP_
