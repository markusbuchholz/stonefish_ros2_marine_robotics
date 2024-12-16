// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from stonefish_ros2:msg/NEDPose.idl
// generated code does not contain a copyright notice

#ifndef STONEFISH_ROS2__MSG__DETAIL__NED_POSE__TRAITS_HPP_
#define STONEFISH_ROS2__MSG__DETAIL__NED_POSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "stonefish_ros2/msg/detail/ned_pose__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace stonefish_ros2
{

namespace msg
{

inline void to_flow_style_yaml(
  const NEDPose & msg,
  std::ostream & out)
{
  out << "{";
  // member: north
  {
    out << "north: ";
    rosidl_generator_traits::value_to_yaml(msg.north, out);
    out << ", ";
  }

  // member: east
  {
    out << "east: ";
    rosidl_generator_traits::value_to_yaml(msg.east, out);
    out << ", ";
  }

  // member: down
  {
    out << "down: ";
    rosidl_generator_traits::value_to_yaml(msg.down, out);
    out << ", ";
  }

  // member: roll
  {
    out << "roll: ";
    rosidl_generator_traits::value_to_yaml(msg.roll, out);
    out << ", ";
  }

  // member: pitch
  {
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NEDPose & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: north
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "north: ";
    rosidl_generator_traits::value_to_yaml(msg.north, out);
    out << "\n";
  }

  // member: east
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "east: ";
    rosidl_generator_traits::value_to_yaml(msg.east, out);
    out << "\n";
  }

  // member: down
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "down: ";
    rosidl_generator_traits::value_to_yaml(msg.down, out);
    out << "\n";
  }

  // member: roll
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "roll: ";
    rosidl_generator_traits::value_to_yaml(msg.roll, out);
    out << "\n";
  }

  // member: pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << "\n";
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NEDPose & msg, bool use_flow_style = false)
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
  const stonefish_ros2::msg::NEDPose & msg,
  std::ostream & out, size_t indentation = 0)
{
  stonefish_ros2::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use stonefish_ros2::msg::to_yaml() instead")]]
inline std::string to_yaml(const stonefish_ros2::msg::NEDPose & msg)
{
  return stonefish_ros2::msg::to_yaml(msg);
}

template<>
inline const char * data_type<stonefish_ros2::msg::NEDPose>()
{
  return "stonefish_ros2::msg::NEDPose";
}

template<>
inline const char * name<stonefish_ros2::msg::NEDPose>()
{
  return "stonefish_ros2/msg/NEDPose";
}

template<>
struct has_fixed_size<stonefish_ros2::msg::NEDPose>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<stonefish_ros2::msg::NEDPose>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<stonefish_ros2::msg::NEDPose>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // STONEFISH_ROS2__MSG__DETAIL__NED_POSE__TRAITS_HPP_
