// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from stonefish_ros2:msg/DVLBeam.idl
// generated code does not contain a copyright notice

#ifndef STONEFISH_ROS2__MSG__DETAIL__DVL_BEAM__TRAITS_HPP_
#define STONEFISH_ROS2__MSG__DETAIL__DVL_BEAM__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "stonefish_ros2/msg/detail/dvl_beam__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"

namespace stonefish_ros2
{

namespace msg
{

inline void to_flow_style_yaml(
  const DVLBeam & msg,
  std::ostream & out)
{
  out << "{";
  // member: range
  {
    out << "range: ";
    rosidl_generator_traits::value_to_yaml(msg.range, out);
    out << ", ";
  }

  // member: range_covariance
  {
    out << "range_covariance: ";
    rosidl_generator_traits::value_to_yaml(msg.range_covariance, out);
    out << ", ";
  }

  // member: velocity
  {
    out << "velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity, out);
    out << ", ";
  }

  // member: velocity_covariance
  {
    out << "velocity_covariance: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity_covariance, out);
    out << ", ";
  }

  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DVLBeam & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: range
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "range: ";
    rosidl_generator_traits::value_to_yaml(msg.range, out);
    out << "\n";
  }

  // member: range_covariance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "range_covariance: ";
    rosidl_generator_traits::value_to_yaml(msg.range_covariance, out);
    out << "\n";
  }

  // member: velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity, out);
    out << "\n";
  }

  // member: velocity_covariance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity_covariance: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity_covariance, out);
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DVLBeam & msg, bool use_flow_style = false)
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
  const stonefish_ros2::msg::DVLBeam & msg,
  std::ostream & out, size_t indentation = 0)
{
  stonefish_ros2::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use stonefish_ros2::msg::to_yaml() instead")]]
inline std::string to_yaml(const stonefish_ros2::msg::DVLBeam & msg)
{
  return stonefish_ros2::msg::to_yaml(msg);
}

template<>
inline const char * data_type<stonefish_ros2::msg::DVLBeam>()
{
  return "stonefish_ros2::msg::DVLBeam";
}

template<>
inline const char * name<stonefish_ros2::msg::DVLBeam>()
{
  return "stonefish_ros2/msg/DVLBeam";
}

template<>
struct has_fixed_size<stonefish_ros2::msg::DVLBeam>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::PoseStamped>::value> {};

template<>
struct has_bounded_size<stonefish_ros2::msg::DVLBeam>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::PoseStamped>::value> {};

template<>
struct is_message<stonefish_ros2::msg::DVLBeam>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // STONEFISH_ROS2__MSG__DETAIL__DVL_BEAM__TRAITS_HPP_
