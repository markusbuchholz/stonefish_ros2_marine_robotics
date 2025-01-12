// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from stonefish_ros2:msg/NEDPose.idl
// generated code does not contain a copyright notice

#ifndef STONEFISH_ROS2__MSG__DETAIL__NED_POSE__BUILDER_HPP_
#define STONEFISH_ROS2__MSG__DETAIL__NED_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "stonefish_ros2/msg/detail/ned_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace stonefish_ros2
{

namespace msg
{

namespace builder
{

class Init_NEDPose_yaw
{
public:
  explicit Init_NEDPose_yaw(::stonefish_ros2::msg::NEDPose & msg)
  : msg_(msg)
  {}
  ::stonefish_ros2::msg::NEDPose yaw(::stonefish_ros2::msg::NEDPose::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::stonefish_ros2::msg::NEDPose msg_;
};

class Init_NEDPose_pitch
{
public:
  explicit Init_NEDPose_pitch(::stonefish_ros2::msg::NEDPose & msg)
  : msg_(msg)
  {}
  Init_NEDPose_yaw pitch(::stonefish_ros2::msg::NEDPose::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_NEDPose_yaw(msg_);
  }

private:
  ::stonefish_ros2::msg::NEDPose msg_;
};

class Init_NEDPose_roll
{
public:
  explicit Init_NEDPose_roll(::stonefish_ros2::msg::NEDPose & msg)
  : msg_(msg)
  {}
  Init_NEDPose_pitch roll(::stonefish_ros2::msg::NEDPose::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_NEDPose_pitch(msg_);
  }

private:
  ::stonefish_ros2::msg::NEDPose msg_;
};

class Init_NEDPose_down
{
public:
  explicit Init_NEDPose_down(::stonefish_ros2::msg::NEDPose & msg)
  : msg_(msg)
  {}
  Init_NEDPose_roll down(::stonefish_ros2::msg::NEDPose::_down_type arg)
  {
    msg_.down = std::move(arg);
    return Init_NEDPose_roll(msg_);
  }

private:
  ::stonefish_ros2::msg::NEDPose msg_;
};

class Init_NEDPose_east
{
public:
  explicit Init_NEDPose_east(::stonefish_ros2::msg::NEDPose & msg)
  : msg_(msg)
  {}
  Init_NEDPose_down east(::stonefish_ros2::msg::NEDPose::_east_type arg)
  {
    msg_.east = std::move(arg);
    return Init_NEDPose_down(msg_);
  }

private:
  ::stonefish_ros2::msg::NEDPose msg_;
};

class Init_NEDPose_north
{
public:
  Init_NEDPose_north()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NEDPose_east north(::stonefish_ros2::msg::NEDPose::_north_type arg)
  {
    msg_.north = std::move(arg);
    return Init_NEDPose_east(msg_);
  }

private:
  ::stonefish_ros2::msg::NEDPose msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::stonefish_ros2::msg::NEDPose>()
{
  return stonefish_ros2::msg::builder::Init_NEDPose_north();
}

}  // namespace stonefish_ros2

#endif  // STONEFISH_ROS2__MSG__DETAIL__NED_POSE__BUILDER_HPP_
