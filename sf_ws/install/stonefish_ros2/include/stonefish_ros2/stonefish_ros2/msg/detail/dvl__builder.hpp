// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from stonefish_ros2:msg/DVL.idl
// generated code does not contain a copyright notice

#ifndef STONEFISH_ROS2__MSG__DETAIL__DVL__BUILDER_HPP_
#define STONEFISH_ROS2__MSG__DETAIL__DVL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "stonefish_ros2/msg/detail/dvl__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace stonefish_ros2
{

namespace msg
{

namespace builder
{

class Init_DVL_beams
{
public:
  explicit Init_DVL_beams(::stonefish_ros2::msg::DVL & msg)
  : msg_(msg)
  {}
  ::stonefish_ros2::msg::DVL beams(::stonefish_ros2::msg::DVL::_beams_type arg)
  {
    msg_.beams = std::move(arg);
    return std::move(msg_);
  }

private:
  ::stonefish_ros2::msg::DVL msg_;
};

class Init_DVL_altitude
{
public:
  explicit Init_DVL_altitude(::stonefish_ros2::msg::DVL & msg)
  : msg_(msg)
  {}
  Init_DVL_beams altitude(::stonefish_ros2::msg::DVL::_altitude_type arg)
  {
    msg_.altitude = std::move(arg);
    return Init_DVL_beams(msg_);
  }

private:
  ::stonefish_ros2::msg::DVL msg_;
};

class Init_DVL_velocity_covariance
{
public:
  explicit Init_DVL_velocity_covariance(::stonefish_ros2::msg::DVL & msg)
  : msg_(msg)
  {}
  Init_DVL_altitude velocity_covariance(::stonefish_ros2::msg::DVL::_velocity_covariance_type arg)
  {
    msg_.velocity_covariance = std::move(arg);
    return Init_DVL_altitude(msg_);
  }

private:
  ::stonefish_ros2::msg::DVL msg_;
};

class Init_DVL_velocity
{
public:
  explicit Init_DVL_velocity(::stonefish_ros2::msg::DVL & msg)
  : msg_(msg)
  {}
  Init_DVL_velocity_covariance velocity(::stonefish_ros2::msg::DVL::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_DVL_velocity_covariance(msg_);
  }

private:
  ::stonefish_ros2::msg::DVL msg_;
};

class Init_DVL_header
{
public:
  Init_DVL_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DVL_velocity header(::stonefish_ros2::msg::DVL::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DVL_velocity(msg_);
  }

private:
  ::stonefish_ros2::msg::DVL msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::stonefish_ros2::msg::DVL>()
{
  return stonefish_ros2::msg::builder::Init_DVL_header();
}

}  // namespace stonefish_ros2

#endif  // STONEFISH_ROS2__MSG__DETAIL__DVL__BUILDER_HPP_
