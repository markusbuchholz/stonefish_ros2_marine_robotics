// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from stonefish_ros2:msg/ThrusterState.idl
// generated code does not contain a copyright notice

#ifndef STONEFISH_ROS2__MSG__DETAIL__THRUSTER_STATE__BUILDER_HPP_
#define STONEFISH_ROS2__MSG__DETAIL__THRUSTER_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "stonefish_ros2/msg/detail/thruster_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace stonefish_ros2
{

namespace msg
{

namespace builder
{

class Init_ThrusterState_torque
{
public:
  explicit Init_ThrusterState_torque(::stonefish_ros2::msg::ThrusterState & msg)
  : msg_(msg)
  {}
  ::stonefish_ros2::msg::ThrusterState torque(::stonefish_ros2::msg::ThrusterState::_torque_type arg)
  {
    msg_.torque = std::move(arg);
    return std::move(msg_);
  }

private:
  ::stonefish_ros2::msg::ThrusterState msg_;
};

class Init_ThrusterState_thrust
{
public:
  explicit Init_ThrusterState_thrust(::stonefish_ros2::msg::ThrusterState & msg)
  : msg_(msg)
  {}
  Init_ThrusterState_torque thrust(::stonefish_ros2::msg::ThrusterState::_thrust_type arg)
  {
    msg_.thrust = std::move(arg);
    return Init_ThrusterState_torque(msg_);
  }

private:
  ::stonefish_ros2::msg::ThrusterState msg_;
};

class Init_ThrusterState_rpm
{
public:
  explicit Init_ThrusterState_rpm(::stonefish_ros2::msg::ThrusterState & msg)
  : msg_(msg)
  {}
  Init_ThrusterState_thrust rpm(::stonefish_ros2::msg::ThrusterState::_rpm_type arg)
  {
    msg_.rpm = std::move(arg);
    return Init_ThrusterState_thrust(msg_);
  }

private:
  ::stonefish_ros2::msg::ThrusterState msg_;
};

class Init_ThrusterState_setpoint
{
public:
  explicit Init_ThrusterState_setpoint(::stonefish_ros2::msg::ThrusterState & msg)
  : msg_(msg)
  {}
  Init_ThrusterState_rpm setpoint(::stonefish_ros2::msg::ThrusterState::_setpoint_type arg)
  {
    msg_.setpoint = std::move(arg);
    return Init_ThrusterState_rpm(msg_);
  }

private:
  ::stonefish_ros2::msg::ThrusterState msg_;
};

class Init_ThrusterState_header
{
public:
  Init_ThrusterState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ThrusterState_setpoint header(::stonefish_ros2::msg::ThrusterState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ThrusterState_setpoint(msg_);
  }

private:
  ::stonefish_ros2::msg::ThrusterState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::stonefish_ros2::msg::ThrusterState>()
{
  return stonefish_ros2::msg::builder::Init_ThrusterState_header();
}

}  // namespace stonefish_ros2

#endif  // STONEFISH_ROS2__MSG__DETAIL__THRUSTER_STATE__BUILDER_HPP_
