// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from stonefish_ros2:msg/BeaconInfo.idl
// generated code does not contain a copyright notice

#ifndef STONEFISH_ROS2__MSG__DETAIL__BEACON_INFO__BUILDER_HPP_
#define STONEFISH_ROS2__MSG__DETAIL__BEACON_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "stonefish_ros2/msg/detail/beacon_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace stonefish_ros2
{

namespace msg
{

namespace builder
{

class Init_BeaconInfo_local_depth
{
public:
  explicit Init_BeaconInfo_local_depth(::stonefish_ros2::msg::BeaconInfo & msg)
  : msg_(msg)
  {}
  ::stonefish_ros2::msg::BeaconInfo local_depth(::stonefish_ros2::msg::BeaconInfo::_local_depth_type arg)
  {
    msg_.local_depth = std::move(arg);
    return std::move(msg_);
  }

private:
  ::stonefish_ros2::msg::BeaconInfo msg_;
};

class Init_BeaconInfo_local_orientation
{
public:
  explicit Init_BeaconInfo_local_orientation(::stonefish_ros2::msg::BeaconInfo & msg)
  : msg_(msg)
  {}
  Init_BeaconInfo_local_depth local_orientation(::stonefish_ros2::msg::BeaconInfo::_local_orientation_type arg)
  {
    msg_.local_orientation = std::move(arg);
    return Init_BeaconInfo_local_depth(msg_);
  }

private:
  ::stonefish_ros2::msg::BeaconInfo msg_;
};

class Init_BeaconInfo_relative_position
{
public:
  explicit Init_BeaconInfo_relative_position(::stonefish_ros2::msg::BeaconInfo & msg)
  : msg_(msg)
  {}
  Init_BeaconInfo_local_orientation relative_position(::stonefish_ros2::msg::BeaconInfo::_relative_position_type arg)
  {
    msg_.relative_position = std::move(arg);
    return Init_BeaconInfo_local_orientation(msg_);
  }

private:
  ::stonefish_ros2::msg::BeaconInfo msg_;
};

class Init_BeaconInfo_elevation
{
public:
  explicit Init_BeaconInfo_elevation(::stonefish_ros2::msg::BeaconInfo & msg)
  : msg_(msg)
  {}
  Init_BeaconInfo_relative_position elevation(::stonefish_ros2::msg::BeaconInfo::_elevation_type arg)
  {
    msg_.elevation = std::move(arg);
    return Init_BeaconInfo_relative_position(msg_);
  }

private:
  ::stonefish_ros2::msg::BeaconInfo msg_;
};

class Init_BeaconInfo_azimuth
{
public:
  explicit Init_BeaconInfo_azimuth(::stonefish_ros2::msg::BeaconInfo & msg)
  : msg_(msg)
  {}
  Init_BeaconInfo_elevation azimuth(::stonefish_ros2::msg::BeaconInfo::_azimuth_type arg)
  {
    msg_.azimuth = std::move(arg);
    return Init_BeaconInfo_elevation(msg_);
  }

private:
  ::stonefish_ros2::msg::BeaconInfo msg_;
};

class Init_BeaconInfo_range
{
public:
  explicit Init_BeaconInfo_range(::stonefish_ros2::msg::BeaconInfo & msg)
  : msg_(msg)
  {}
  Init_BeaconInfo_azimuth range(::stonefish_ros2::msg::BeaconInfo::_range_type arg)
  {
    msg_.range = std::move(arg);
    return Init_BeaconInfo_azimuth(msg_);
  }

private:
  ::stonefish_ros2::msg::BeaconInfo msg_;
};

class Init_BeaconInfo_beacon_id
{
public:
  explicit Init_BeaconInfo_beacon_id(::stonefish_ros2::msg::BeaconInfo & msg)
  : msg_(msg)
  {}
  Init_BeaconInfo_range beacon_id(::stonefish_ros2::msg::BeaconInfo::_beacon_id_type arg)
  {
    msg_.beacon_id = std::move(arg);
    return Init_BeaconInfo_range(msg_);
  }

private:
  ::stonefish_ros2::msg::BeaconInfo msg_;
};

class Init_BeaconInfo_header
{
public:
  Init_BeaconInfo_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BeaconInfo_beacon_id header(::stonefish_ros2::msg::BeaconInfo::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_BeaconInfo_beacon_id(msg_);
  }

private:
  ::stonefish_ros2::msg::BeaconInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::stonefish_ros2::msg::BeaconInfo>()
{
  return stonefish_ros2::msg::builder::Init_BeaconInfo_header();
}

}  // namespace stonefish_ros2

#endif  // STONEFISH_ROS2__MSG__DETAIL__BEACON_INFO__BUILDER_HPP_
