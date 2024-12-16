// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from cola2_msgs:msg/CaptainStatus.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__CAPTAIN_STATUS__BUILDER_HPP_
#define COLA2_MSGS__MSG__DETAIL__CAPTAIN_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "cola2_msgs/msg/detail/captain_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace cola2_msgs
{

namespace msg
{

namespace builder
{

class Init_CaptainStatus_loaded_missions
{
public:
  explicit Init_CaptainStatus_loaded_missions(::cola2_msgs::msg::CaptainStatus & msg)
  : msg_(msg)
  {}
  ::cola2_msgs::msg::CaptainStatus loaded_missions(::cola2_msgs::msg::CaptainStatus::_loaded_missions_type arg)
  {
    msg_.loaded_missions = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cola2_msgs::msg::CaptainStatus msg_;
};

class Init_CaptainStatus_message
{
public:
  explicit Init_CaptainStatus_message(::cola2_msgs::msg::CaptainStatus & msg)
  : msg_(msg)
  {}
  Init_CaptainStatus_loaded_missions message(::cola2_msgs::msg::CaptainStatus::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_CaptainStatus_loaded_missions(msg_);
  }

private:
  ::cola2_msgs::msg::CaptainStatus msg_;
};

class Init_CaptainStatus_state
{
public:
  explicit Init_CaptainStatus_state(::cola2_msgs::msg::CaptainStatus & msg)
  : msg_(msg)
  {}
  Init_CaptainStatus_message state(::cola2_msgs::msg::CaptainStatus::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_CaptainStatus_message(msg_);
  }

private:
  ::cola2_msgs::msg::CaptainStatus msg_;
};

class Init_CaptainStatus_header
{
public:
  Init_CaptainStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CaptainStatus_state header(::cola2_msgs::msg::CaptainStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_CaptainStatus_state(msg_);
  }

private:
  ::cola2_msgs::msg::CaptainStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::cola2_msgs::msg::CaptainStatus>()
{
  return cola2_msgs::msg::builder::Init_CaptainStatus_header();
}

}  // namespace cola2_msgs

#endif  // COLA2_MSGS__MSG__DETAIL__CAPTAIN_STATUS__BUILDER_HPP_
