// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from cola2_msgs:msg/MissionState.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__MISSION_STATE__BUILDER_HPP_
#define COLA2_MSGS__MSG__DETAIL__MISSION_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "cola2_msgs/msg/detail/mission_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace cola2_msgs
{

namespace msg
{

namespace builder
{

class Init_MissionState_last_active
{
public:
  explicit Init_MissionState_last_active(::cola2_msgs::msg::MissionState & msg)
  : msg_(msg)
  {}
  ::cola2_msgs::msg::MissionState last_active(::cola2_msgs::msg::MissionState::_last_active_type arg)
  {
    msg_.last_active = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cola2_msgs::msg::MissionState msg_;
};

class Init_MissionState_current_step
{
public:
  explicit Init_MissionState_current_step(::cola2_msgs::msg::MissionState & msg)
  : msg_(msg)
  {}
  Init_MissionState_last_active current_step(::cola2_msgs::msg::MissionState::_current_step_type arg)
  {
    msg_.current_step = std::move(arg);
    return Init_MissionState_last_active(msg_);
  }

private:
  ::cola2_msgs::msg::MissionState msg_;
};

class Init_MissionState_name
{
public:
  Init_MissionState_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MissionState_current_step name(::cola2_msgs::msg::MissionState::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_MissionState_current_step(msg_);
  }

private:
  ::cola2_msgs::msg::MissionState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::cola2_msgs::msg::MissionState>()
{
  return cola2_msgs::msg::builder::Init_MissionState_name();
}

}  // namespace cola2_msgs

#endif  // COLA2_MSGS__MSG__DETAIL__MISSION_STATE__BUILDER_HPP_
