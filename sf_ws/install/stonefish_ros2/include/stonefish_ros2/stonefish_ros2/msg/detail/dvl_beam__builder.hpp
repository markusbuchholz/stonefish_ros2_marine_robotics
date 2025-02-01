// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from stonefish_ros2:msg/DVLBeam.idl
// generated code does not contain a copyright notice

#ifndef STONEFISH_ROS2__MSG__DETAIL__DVL_BEAM__BUILDER_HPP_
#define STONEFISH_ROS2__MSG__DETAIL__DVL_BEAM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "stonefish_ros2/msg/detail/dvl_beam__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace stonefish_ros2
{

namespace msg
{

namespace builder
{

class Init_DVLBeam_pose
{
public:
  explicit Init_DVLBeam_pose(::stonefish_ros2::msg::DVLBeam & msg)
  : msg_(msg)
  {}
  ::stonefish_ros2::msg::DVLBeam pose(::stonefish_ros2::msg::DVLBeam::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::stonefish_ros2::msg::DVLBeam msg_;
};

class Init_DVLBeam_velocity_covariance
{
public:
  explicit Init_DVLBeam_velocity_covariance(::stonefish_ros2::msg::DVLBeam & msg)
  : msg_(msg)
  {}
  Init_DVLBeam_pose velocity_covariance(::stonefish_ros2::msg::DVLBeam::_velocity_covariance_type arg)
  {
    msg_.velocity_covariance = std::move(arg);
    return Init_DVLBeam_pose(msg_);
  }

private:
  ::stonefish_ros2::msg::DVLBeam msg_;
};

class Init_DVLBeam_velocity
{
public:
  explicit Init_DVLBeam_velocity(::stonefish_ros2::msg::DVLBeam & msg)
  : msg_(msg)
  {}
  Init_DVLBeam_velocity_covariance velocity(::stonefish_ros2::msg::DVLBeam::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_DVLBeam_velocity_covariance(msg_);
  }

private:
  ::stonefish_ros2::msg::DVLBeam msg_;
};

class Init_DVLBeam_range_covariance
{
public:
  explicit Init_DVLBeam_range_covariance(::stonefish_ros2::msg::DVLBeam & msg)
  : msg_(msg)
  {}
  Init_DVLBeam_velocity range_covariance(::stonefish_ros2::msg::DVLBeam::_range_covariance_type arg)
  {
    msg_.range_covariance = std::move(arg);
    return Init_DVLBeam_velocity(msg_);
  }

private:
  ::stonefish_ros2::msg::DVLBeam msg_;
};

class Init_DVLBeam_range
{
public:
  Init_DVLBeam_range()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DVLBeam_range_covariance range(::stonefish_ros2::msg::DVLBeam::_range_type arg)
  {
    msg_.range = std::move(arg);
    return Init_DVLBeam_range_covariance(msg_);
  }

private:
  ::stonefish_ros2::msg::DVLBeam msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::stonefish_ros2::msg::DVLBeam>()
{
  return stonefish_ros2::msg::builder::Init_DVLBeam_range();
}

}  // namespace stonefish_ros2

#endif  // STONEFISH_ROS2__MSG__DETAIL__DVL_BEAM__BUILDER_HPP_
