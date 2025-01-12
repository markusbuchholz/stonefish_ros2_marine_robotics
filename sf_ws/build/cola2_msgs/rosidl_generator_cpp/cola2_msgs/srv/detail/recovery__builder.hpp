// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from cola2_msgs:srv/Recovery.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__SRV__DETAIL__RECOVERY__BUILDER_HPP_
#define COLA2_MSGS__SRV__DETAIL__RECOVERY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "cola2_msgs/srv/detail/recovery__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace cola2_msgs
{

namespace srv
{

namespace builder
{

class Init_Recovery_Request_requested_action
{
public:
  Init_Recovery_Request_requested_action()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::cola2_msgs::srv::Recovery_Request requested_action(::cola2_msgs::srv::Recovery_Request::_requested_action_type arg)
  {
    msg_.requested_action = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cola2_msgs::srv::Recovery_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::cola2_msgs::srv::Recovery_Request>()
{
  return cola2_msgs::srv::builder::Init_Recovery_Request_requested_action();
}

}  // namespace cola2_msgs


namespace cola2_msgs
{

namespace srv
{

namespace builder
{

class Init_Recovery_Response_attempted
{
public:
  Init_Recovery_Response_attempted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::cola2_msgs::srv::Recovery_Response attempted(::cola2_msgs::srv::Recovery_Response::_attempted_type arg)
  {
    msg_.attempted = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cola2_msgs::srv::Recovery_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::cola2_msgs::srv::Recovery_Response>()
{
  return cola2_msgs::srv::builder::Init_Recovery_Response_attempted();
}

}  // namespace cola2_msgs

#endif  // COLA2_MSGS__SRV__DETAIL__RECOVERY__BUILDER_HPP_
