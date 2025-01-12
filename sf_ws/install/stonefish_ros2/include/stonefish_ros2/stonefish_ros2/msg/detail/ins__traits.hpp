// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from stonefish_ros2:msg/INS.idl
// generated code does not contain a copyright notice

#ifndef STONEFISH_ROS2__MSG__DETAIL__INS__TRAITS_HPP_
#define STONEFISH_ROS2__MSG__DETAIL__INS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "stonefish_ros2/msg/detail/ins__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'pose'
// Member 'pose_variance'
#include "stonefish_ros2/msg/detail/ned_pose__traits.hpp"
// Member 'body_velocity'
// Member 'rpy_rate'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace stonefish_ros2
{

namespace msg
{

inline void to_flow_style_yaml(
  const INS & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: latitude
  {
    out << "latitude: ";
    rosidl_generator_traits::value_to_yaml(msg.latitude, out);
    out << ", ";
  }

  // member: longitude
  {
    out << "longitude: ";
    rosidl_generator_traits::value_to_yaml(msg.longitude, out);
    out << ", ";
  }

  // member: origin_latitude
  {
    out << "origin_latitude: ";
    rosidl_generator_traits::value_to_yaml(msg.origin_latitude, out);
    out << ", ";
  }

  // member: origin_longitude
  {
    out << "origin_longitude: ";
    rosidl_generator_traits::value_to_yaml(msg.origin_longitude, out);
    out << ", ";
  }

  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
    out << ", ";
  }

  // member: pose_variance
  {
    out << "pose_variance: ";
    to_flow_style_yaml(msg.pose_variance, out);
    out << ", ";
  }

  // member: altitude
  {
    out << "altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.altitude, out);
    out << ", ";
  }

  // member: body_velocity
  {
    out << "body_velocity: ";
    to_flow_style_yaml(msg.body_velocity, out);
    out << ", ";
  }

  // member: rpy_rate
  {
    out << "rpy_rate: ";
    to_flow_style_yaml(msg.rpy_rate, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const INS & msg,
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

  // member: latitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "latitude: ";
    rosidl_generator_traits::value_to_yaml(msg.latitude, out);
    out << "\n";
  }

  // member: longitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "longitude: ";
    rosidl_generator_traits::value_to_yaml(msg.longitude, out);
    out << "\n";
  }

  // member: origin_latitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "origin_latitude: ";
    rosidl_generator_traits::value_to_yaml(msg.origin_latitude, out);
    out << "\n";
  }

  // member: origin_longitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "origin_longitude: ";
    rosidl_generator_traits::value_to_yaml(msg.origin_longitude, out);
    out << "\n";
  }

  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_block_style_yaml(msg.pose, out, indentation + 2);
  }

  // member: pose_variance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose_variance:\n";
    to_block_style_yaml(msg.pose_variance, out, indentation + 2);
  }

  // member: altitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.altitude, out);
    out << "\n";
  }

  // member: body_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "body_velocity:\n";
    to_block_style_yaml(msg.body_velocity, out, indentation + 2);
  }

  // member: rpy_rate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rpy_rate:\n";
    to_block_style_yaml(msg.rpy_rate, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const INS & msg, bool use_flow_style = false)
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
  const stonefish_ros2::msg::INS & msg,
  std::ostream & out, size_t indentation = 0)
{
  stonefish_ros2::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use stonefish_ros2::msg::to_yaml() instead")]]
inline std::string to_yaml(const stonefish_ros2::msg::INS & msg)
{
  return stonefish_ros2::msg::to_yaml(msg);
}

template<>
inline const char * data_type<stonefish_ros2::msg::INS>()
{
  return "stonefish_ros2::msg::INS";
}

template<>
inline const char * name<stonefish_ros2::msg::INS>()
{
  return "stonefish_ros2/msg/INS";
}

template<>
struct has_fixed_size<stonefish_ros2::msg::INS>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Vector3>::value && has_fixed_size<std_msgs::msg::Header>::value && has_fixed_size<stonefish_ros2::msg::NEDPose>::value> {};

template<>
struct has_bounded_size<stonefish_ros2::msg::INS>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Vector3>::value && has_bounded_size<std_msgs::msg::Header>::value && has_bounded_size<stonefish_ros2::msg::NEDPose>::value> {};

template<>
struct is_message<stonefish_ros2::msg::INS>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // STONEFISH_ROS2__MSG__DETAIL__INS__TRAITS_HPP_
