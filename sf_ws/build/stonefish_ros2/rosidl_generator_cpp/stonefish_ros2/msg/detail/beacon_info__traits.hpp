// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from stonefish_ros2:msg/BeaconInfo.idl
// generated code does not contain a copyright notice

#ifndef STONEFISH_ROS2__MSG__DETAIL__BEACON_INFO__TRAITS_HPP_
#define STONEFISH_ROS2__MSG__DETAIL__BEACON_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "stonefish_ros2/msg/detail/beacon_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'relative_position'
#include "geometry_msgs/msg/detail/point__traits.hpp"
// Member 'local_orientation'
#include "geometry_msgs/msg/detail/quaternion__traits.hpp"

namespace stonefish_ros2
{

namespace msg
{

inline void to_flow_style_yaml(
  const BeaconInfo & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: beacon_id
  {
    out << "beacon_id: ";
    rosidl_generator_traits::value_to_yaml(msg.beacon_id, out);
    out << ", ";
  }

  // member: range
  {
    out << "range: ";
    rosidl_generator_traits::value_to_yaml(msg.range, out);
    out << ", ";
  }

  // member: azimuth
  {
    out << "azimuth: ";
    rosidl_generator_traits::value_to_yaml(msg.azimuth, out);
    out << ", ";
  }

  // member: elevation
  {
    out << "elevation: ";
    rosidl_generator_traits::value_to_yaml(msg.elevation, out);
    out << ", ";
  }

  // member: relative_position
  {
    out << "relative_position: ";
    to_flow_style_yaml(msg.relative_position, out);
    out << ", ";
  }

  // member: local_orientation
  {
    out << "local_orientation: ";
    to_flow_style_yaml(msg.local_orientation, out);
    out << ", ";
  }

  // member: local_depth
  {
    out << "local_depth: ";
    rosidl_generator_traits::value_to_yaml(msg.local_depth, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BeaconInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: beacon_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "beacon_id: ";
    rosidl_generator_traits::value_to_yaml(msg.beacon_id, out);
    out << "\n";
  }

  // member: range
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "range: ";
    rosidl_generator_traits::value_to_yaml(msg.range, out);
    out << "\n";
  }

  // member: azimuth
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "azimuth: ";
    rosidl_generator_traits::value_to_yaml(msg.azimuth, out);
    out << "\n";
  }

  // member: elevation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "elevation: ";
    rosidl_generator_traits::value_to_yaml(msg.elevation, out);
    out << "\n";
  }

  // member: relative_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "relative_position:\n";
    to_block_style_yaml(msg.relative_position, out, indentation + 2);
  }

  // member: local_orientation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "local_orientation:\n";
    to_block_style_yaml(msg.local_orientation, out, indentation + 2);
  }

  // member: local_depth
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "local_depth: ";
    rosidl_generator_traits::value_to_yaml(msg.local_depth, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BeaconInfo & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace stonefish_ros2

namespace rosidl_generator_traits
{

[[deprecated("use stonefish_ros2::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const stonefish_ros2::msg::BeaconInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  stonefish_ros2::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use stonefish_ros2::msg::to_yaml() instead")]]
inline std::string to_yaml(const stonefish_ros2::msg::BeaconInfo & msg)
{
  return stonefish_ros2::msg::to_yaml(msg);
}

template<>
inline const char * data_type<stonefish_ros2::msg::BeaconInfo>()
{
  return "stonefish_ros2::msg::BeaconInfo";
}

template<>
inline const char * name<stonefish_ros2::msg::BeaconInfo>()
{
  return "stonefish_ros2/msg/BeaconInfo";
}

template<>
struct has_fixed_size<stonefish_ros2::msg::BeaconInfo>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Point>::value && has_fixed_size<geometry_msgs::msg::Quaternion>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<stonefish_ros2::msg::BeaconInfo>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Point>::value && has_bounded_size<geometry_msgs::msg::Quaternion>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<stonefish_ros2::msg::BeaconInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // STONEFISH_ROS2__MSG__DETAIL__BEACON_INFO__TRAITS_HPP_
