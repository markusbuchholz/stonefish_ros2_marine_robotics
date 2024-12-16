// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from cola2_msgs:srv/DigitalOutput.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__SRV__DETAIL__DIGITAL_OUTPUT__BUILDER_HPP_
#define COLA2_MSGS__SRV__DETAIL__DIGITAL_OUTPUT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "cola2_msgs/srv/detail/digital_output__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace cola2_msgs
{

namespace srv
{

namespace builder
{

class Init_DigitalOutput_Request_value
{
public:
  explicit Init_DigitalOutput_Request_value(::cola2_msgs::srv::DigitalOutput_Request & msg)
  : msg_(msg)
  {}
  ::cola2_msgs::srv::DigitalOutput_Request value(::cola2_msgs::srv::DigitalOutput_Request::_value_type arg)
  {
    msg_.value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cola2_msgs::srv::DigitalOutput_Request msg_;
};

class Init_DigitalOutput_Request_digital_output
{
public:
  Init_DigitalOutput_Request_digital_output()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DigitalOutput_Request_value digital_output(::cola2_msgs::srv::DigitalOutput_Request::_digital_output_type arg)
  {
    msg_.digital_output = std::move(arg);
    return Init_DigitalOutput_Request_value(msg_);
  }

private:
  ::cola2_msgs::srv::DigitalOutput_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::cola2_msgs::srv::DigitalOutput_Request>()
{
  return cola2_msgs::srv::builder::Init_DigitalOutput_Request_digital_output();
}

}  // namespace cola2_msgs


namespace cola2_msgs
{

namespace srv
{

namespace builder
{

class Init_DigitalOutput_Response_message
{
public:
  explicit Init_DigitalOutput_Response_message(::cola2_msgs::srv::DigitalOutput_Response & msg)
  : msg_(msg)
  {}
  ::cola2_msgs::srv::DigitalOutput_Response message(::cola2_msgs::srv::DigitalOutput_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cola2_msgs::srv::DigitalOutput_Response msg_;
};

class Init_DigitalOutput_Response_success
{
public:
  Init_DigitalOutput_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DigitalOutput_Response_message success(::cola2_msgs::srv::DigitalOutput_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_DigitalOutput_Response_message(msg_);
  }

private:
  ::cola2_msgs::srv::DigitalOutput_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::cola2_msgs::srv::DigitalOutput_Response>()
{
  return cola2_msgs::srv::builder::Init_DigitalOutput_Response_success();
}

}  // namespace cola2_msgs

#endif  // COLA2_MSGS__SRV__DETAIL__DIGITAL_OUTPUT__BUILDER_HPP_
