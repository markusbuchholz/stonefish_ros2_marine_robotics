// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from stonefish_ros2:srv/SonarSettings2.idl
// generated code does not contain a copyright notice

#ifndef STONEFISH_ROS2__SRV__DETAIL__SONAR_SETTINGS2__BUILDER_HPP_
#define STONEFISH_ROS2__SRV__DETAIL__SONAR_SETTINGS2__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "stonefish_ros2/srv/detail/sonar_settings2__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace stonefish_ros2
{

namespace srv
{

namespace builder
{

class Init_SonarSettings2_Request_gain
{
public:
  explicit Init_SonarSettings2_Request_gain(::stonefish_ros2::srv::SonarSettings2_Request & msg)
  : msg_(msg)
  {}
  ::stonefish_ros2::srv::SonarSettings2_Request gain(::stonefish_ros2::srv::SonarSettings2_Request::_gain_type arg)
  {
    msg_.gain = std::move(arg);
    return std::move(msg_);
  }

private:
  ::stonefish_ros2::srv::SonarSettings2_Request msg_;
};

class Init_SonarSettings2_Request_rotation_max
{
public:
  explicit Init_SonarSettings2_Request_rotation_max(::stonefish_ros2::srv::SonarSettings2_Request & msg)
  : msg_(msg)
  {}
  Init_SonarSettings2_Request_gain rotation_max(::stonefish_ros2::srv::SonarSettings2_Request::_rotation_max_type arg)
  {
    msg_.rotation_max = std::move(arg);
    return Init_SonarSettings2_Request_gain(msg_);
  }

private:
  ::stonefish_ros2::srv::SonarSettings2_Request msg_;
};

class Init_SonarSettings2_Request_rotation_min
{
public:
  explicit Init_SonarSettings2_Request_rotation_min(::stonefish_ros2::srv::SonarSettings2_Request & msg)
  : msg_(msg)
  {}
  Init_SonarSettings2_Request_rotation_max rotation_min(::stonefish_ros2::srv::SonarSettings2_Request::_rotation_min_type arg)
  {
    msg_.rotation_min = std::move(arg);
    return Init_SonarSettings2_Request_rotation_max(msg_);
  }

private:
  ::stonefish_ros2::srv::SonarSettings2_Request msg_;
};

class Init_SonarSettings2_Request_range_max
{
public:
  explicit Init_SonarSettings2_Request_range_max(::stonefish_ros2::srv::SonarSettings2_Request & msg)
  : msg_(msg)
  {}
  Init_SonarSettings2_Request_rotation_min range_max(::stonefish_ros2::srv::SonarSettings2_Request::_range_max_type arg)
  {
    msg_.range_max = std::move(arg);
    return Init_SonarSettings2_Request_rotation_min(msg_);
  }

private:
  ::stonefish_ros2::srv::SonarSettings2_Request msg_;
};

class Init_SonarSettings2_Request_range_min
{
public:
  Init_SonarSettings2_Request_range_min()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SonarSettings2_Request_range_max range_min(::stonefish_ros2::srv::SonarSettings2_Request::_range_min_type arg)
  {
    msg_.range_min = std::move(arg);
    return Init_SonarSettings2_Request_range_max(msg_);
  }

private:
  ::stonefish_ros2::srv::SonarSettings2_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::stonefish_ros2::srv::SonarSettings2_Request>()
{
  return stonefish_ros2::srv::builder::Init_SonarSettings2_Request_range_min();
}

}  // namespace stonefish_ros2


namespace stonefish_ros2
{

namespace srv
{

namespace builder
{

class Init_SonarSettings2_Response_message
{
public:
  explicit Init_SonarSettings2_Response_message(::stonefish_ros2::srv::SonarSettings2_Response & msg)
  : msg_(msg)
  {}
  ::stonefish_ros2::srv::SonarSettings2_Response message(::stonefish_ros2::srv::SonarSettings2_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::stonefish_ros2::srv::SonarSettings2_Response msg_;
};

class Init_SonarSettings2_Response_success
{
public:
  Init_SonarSettings2_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SonarSettings2_Response_message success(::stonefish_ros2::srv::SonarSettings2_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SonarSettings2_Response_message(msg_);
  }

private:
  ::stonefish_ros2::srv::SonarSettings2_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::stonefish_ros2::srv::SonarSettings2_Response>()
{
  return stonefish_ros2::srv::builder::Init_SonarSettings2_Response_success();
}

}  // namespace stonefish_ros2

#endif  // STONEFISH_ROS2__SRV__DETAIL__SONAR_SETTINGS2__BUILDER_HPP_
