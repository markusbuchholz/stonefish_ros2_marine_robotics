// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from stonefish_ros2:msg/INS.idl
// generated code does not contain a copyright notice

#ifndef STONEFISH_ROS2__MSG__DETAIL__INS__BUILDER_HPP_
#define STONEFISH_ROS2__MSG__DETAIL__INS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "stonefish_ros2/msg/detail/ins__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace stonefish_ros2
{

namespace msg
{

namespace builder
{

class Init_INS_rpy_rate
{
public:
  explicit Init_INS_rpy_rate(::stonefish_ros2::msg::INS & msg)
  : msg_(msg)
  {}
  ::stonefish_ros2::msg::INS rpy_rate(::stonefish_ros2::msg::INS::_rpy_rate_type arg)
  {
    msg_.rpy_rate = std::move(arg);
    return std::move(msg_);
  }

private:
  ::stonefish_ros2::msg::INS msg_;
};

class Init_INS_body_velocity
{
public:
  explicit Init_INS_body_velocity(::stonefish_ros2::msg::INS & msg)
  : msg_(msg)
  {}
  Init_INS_rpy_rate body_velocity(::stonefish_ros2::msg::INS::_body_velocity_type arg)
  {
    msg_.body_velocity = std::move(arg);
    return Init_INS_rpy_rate(msg_);
  }

private:
  ::stonefish_ros2::msg::INS msg_;
};

class Init_INS_altitude
{
public:
  explicit Init_INS_altitude(::stonefish_ros2::msg::INS & msg)
  : msg_(msg)
  {}
  Init_INS_body_velocity altitude(::stonefish_ros2::msg::INS::_altitude_type arg)
  {
    msg_.altitude = std::move(arg);
    return Init_INS_body_velocity(msg_);
  }

private:
  ::stonefish_ros2::msg::INS msg_;
};

class Init_INS_pose_variance
{
public:
  explicit Init_INS_pose_variance(::stonefish_ros2::msg::INS & msg)
  : msg_(msg)
  {}
  Init_INS_altitude pose_variance(::stonefish_ros2::msg::INS::_pose_variance_type arg)
  {
    msg_.pose_variance = std::move(arg);
    return Init_INS_altitude(msg_);
  }

private:
  ::stonefish_ros2::msg::INS msg_;
};

class Init_INS_pose
{
public:
  explicit Init_INS_pose(::stonefish_ros2::msg::INS & msg)
  : msg_(msg)
  {}
  Init_INS_pose_variance pose(::stonefish_ros2::msg::INS::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_INS_pose_variance(msg_);
  }

private:
  ::stonefish_ros2::msg::INS msg_;
};

class Init_INS_origin_longitude
{
public:
  explicit Init_INS_origin_longitude(::stonefish_ros2::msg::INS & msg)
  : msg_(msg)
  {}
  Init_INS_pose origin_longitude(::stonefish_ros2::msg::INS::_origin_longitude_type arg)
  {
    msg_.origin_longitude = std::move(arg);
    return Init_INS_pose(msg_);
  }

private:
  ::stonefish_ros2::msg::INS msg_;
};

class Init_INS_origin_latitude
{
public:
  explicit Init_INS_origin_latitude(::stonefish_ros2::msg::INS & msg)
  : msg_(msg)
  {}
  Init_INS_origin_longitude origin_latitude(::stonefish_ros2::msg::INS::_origin_latitude_type arg)
  {
    msg_.origin_latitude = std::move(arg);
    return Init_INS_origin_longitude(msg_);
  }

private:
  ::stonefish_ros2::msg::INS msg_;
};

class Init_INS_longitude
{
public:
  explicit Init_INS_longitude(::stonefish_ros2::msg::INS & msg)
  : msg_(msg)
  {}
  Init_INS_origin_latitude longitude(::stonefish_ros2::msg::INS::_longitude_type arg)
  {
    msg_.longitude = std::move(arg);
    return Init_INS_origin_latitude(msg_);
  }

private:
  ::stonefish_ros2::msg::INS msg_;
};

class Init_INS_latitude
{
public:
  explicit Init_INS_latitude(::stonefish_ros2::msg::INS & msg)
  : msg_(msg)
  {}
  Init_INS_longitude latitude(::stonefish_ros2::msg::INS::_latitude_type arg)
  {
    msg_.latitude = std::move(arg);
    return Init_INS_longitude(msg_);
  }

private:
  ::stonefish_ros2::msg::INS msg_;
};

class Init_INS_header
{
public:
  Init_INS_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_INS_latitude header(::stonefish_ros2::msg::INS::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_INS_latitude(msg_);
  }

private:
  ::stonefish_ros2::msg::INS msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::stonefish_ros2::msg::INS>()
{
  return stonefish_ros2::msg::builder::Init_INS_header();
}

}  // namespace stonefish_ros2

#endif  // STONEFISH_ROS2__MSG__DETAIL__INS__BUILDER_HPP_
