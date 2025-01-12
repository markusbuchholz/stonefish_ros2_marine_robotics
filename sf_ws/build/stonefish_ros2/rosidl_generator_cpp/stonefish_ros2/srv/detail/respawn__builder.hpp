// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from stonefish_ros2:srv/Respawn.idl
// generated code does not contain a copyright notice

#ifndef STONEFISH_ROS2__SRV__DETAIL__RESPAWN__BUILDER_HPP_
#define STONEFISH_ROS2__SRV__DETAIL__RESPAWN__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "stonefish_ros2/srv/detail/respawn__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace stonefish_ros2
{

namespace srv
{

namespace builder
{

class Init_Respawn_Request_origin
{
public:
  explicit Init_Respawn_Request_origin(::stonefish_ros2::srv::Respawn_Request & msg)
  : msg_(msg)
  {}
  ::stonefish_ros2::srv::Respawn_Request origin(::stonefish_ros2::srv::Respawn_Request::_origin_type arg)
  {
    msg_.origin = std::move(arg);
    return std::move(msg_);
  }

private:
  ::stonefish_ros2::srv::Respawn_Request msg_;
};

class Init_Respawn_Request_name
{
public:
  Init_Respawn_Request_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Respawn_Request_origin name(::stonefish_ros2::srv::Respawn_Request::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_Respawn_Request_origin(msg_);
  }

private:
  ::stonefish_ros2::srv::Respawn_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::stonefish_ros2::srv::Respawn_Request>()
{
  return stonefish_ros2::srv::builder::Init_Respawn_Request_name();
}

}  // namespace stonefish_ros2


namespace stonefish_ros2
{

namespace srv
{

namespace builder
{

class Init_Respawn_Response_message
{
public:
  explicit Init_Respawn_Response_message(::stonefish_ros2::srv::Respawn_Response & msg)
  : msg_(msg)
  {}
  ::stonefish_ros2::srv::Respawn_Response message(::stonefish_ros2::srv::Respawn_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::stonefish_ros2::srv::Respawn_Response msg_;
};

class Init_Respawn_Response_success
{
public:
  Init_Respawn_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Respawn_Response_message success(::stonefish_ros2::srv::Respawn_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_Respawn_Response_message(msg_);
  }

private:
  ::stonefish_ros2::srv::Respawn_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::stonefish_ros2::srv::Respawn_Response>()
{
  return stonefish_ros2::srv::builder::Init_Respawn_Response_success();
}

}  // namespace stonefish_ros2

#endif  // STONEFISH_ROS2__SRV__DETAIL__RESPAWN__BUILDER_HPP_
