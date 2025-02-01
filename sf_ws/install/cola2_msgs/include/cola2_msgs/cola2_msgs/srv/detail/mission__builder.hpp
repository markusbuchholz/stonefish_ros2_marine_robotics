// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from cola2_msgs:srv/Mission.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__SRV__DETAIL__MISSION__BUILDER_HPP_
#define COLA2_MSGS__SRV__DETAIL__MISSION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "cola2_msgs/srv/detail/mission__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace cola2_msgs
{

namespace srv
{

namespace builder
{

class Init_Mission_Request_mission
{
public:
  Init_Mission_Request_mission()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::cola2_msgs::srv::Mission_Request mission(::cola2_msgs::srv::Mission_Request::_mission_type arg)
  {
    msg_.mission = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cola2_msgs::srv::Mission_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::cola2_msgs::srv::Mission_Request>()
{
  return cola2_msgs::srv::builder::Init_Mission_Request_mission();
}

}  // namespace cola2_msgs


namespace cola2_msgs
{

namespace srv
{

namespace builder
{

class Init_Mission_Response_message
{
public:
  explicit Init_Mission_Response_message(::cola2_msgs::srv::Mission_Response & msg)
  : msg_(msg)
  {}
  ::cola2_msgs::srv::Mission_Response message(::cola2_msgs::srv::Mission_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cola2_msgs::srv::Mission_Response msg_;
};

class Init_Mission_Response_success
{
public:
  Init_Mission_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Mission_Response_message success(::cola2_msgs::srv::Mission_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_Mission_Response_message(msg_);
  }

private:
  ::cola2_msgs::srv::Mission_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::cola2_msgs::srv::Mission_Response>()
{
  return cola2_msgs::srv::builder::Init_Mission_Response_success();
}

}  // namespace cola2_msgs

#endif  // COLA2_MSGS__SRV__DETAIL__MISSION__BUILDER_HPP_
