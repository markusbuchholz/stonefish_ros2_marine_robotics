// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from cola2_msgs:srv/MaxJoyVelocity.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__SRV__DETAIL__MAX_JOY_VELOCITY__BUILDER_HPP_
#define COLA2_MSGS__SRV__DETAIL__MAX_JOY_VELOCITY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "cola2_msgs/srv/detail/max_joy_velocity__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace cola2_msgs
{

namespace srv
{

namespace builder
{

class Init_MaxJoyVelocity_Request_max_joy_velocity
{
public:
  Init_MaxJoyVelocity_Request_max_joy_velocity()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::cola2_msgs::srv::MaxJoyVelocity_Request max_joy_velocity(::cola2_msgs::srv::MaxJoyVelocity_Request::_max_joy_velocity_type arg)
  {
    msg_.max_joy_velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cola2_msgs::srv::MaxJoyVelocity_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::cola2_msgs::srv::MaxJoyVelocity_Request>()
{
  return cola2_msgs::srv::builder::Init_MaxJoyVelocity_Request_max_joy_velocity();
}

}  // namespace cola2_msgs


namespace cola2_msgs
{

namespace srv
{

namespace builder
{

class Init_MaxJoyVelocity_Response_attempted
{
public:
  Init_MaxJoyVelocity_Response_attempted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::cola2_msgs::srv::MaxJoyVelocity_Response attempted(::cola2_msgs::srv::MaxJoyVelocity_Response::_attempted_type arg)
  {
    msg_.attempted = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cola2_msgs::srv::MaxJoyVelocity_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::cola2_msgs::srv::MaxJoyVelocity_Response>()
{
  return cola2_msgs::srv::builder::Init_MaxJoyVelocity_Response_attempted();
}

}  // namespace cola2_msgs

#endif  // COLA2_MSGS__SRV__DETAIL__MAX_JOY_VELOCITY__BUILDER_HPP_
