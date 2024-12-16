// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from cola2_msgs:msg/Landmark.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__LANDMARK__TRAITS_HPP_
#define COLA2_MSGS__MSG__DETAIL__LANDMARK__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "cola2_msgs/msg/detail/landmark__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'last_update'
#include "builtin_interfaces/msg/detail/time__traits.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_with_covariance__traits.hpp"

namespace cola2_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Landmark & msg,
  std::ostream & out)
{
  out << "{";
  // member: last_update
  {
    out << "last_update: ";
    to_flow_style_yaml(msg.last_update, out);
    out << ", ";
  }

  // member: landmark_id
  {
    out << "landmark_id: ";
    rosidl_generator_traits::value_to_yaml(msg.landmark_id, out);
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
  const Landmark & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: last_update
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "last_update:\n";
    to_block_style_yaml(msg.last_update, out, indentation + 2);
  }

  // member: landmark_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "landmark_id: ";
    rosidl_generator_traits::value_to_yaml(msg.landmark_id, out);
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

inline std::string to_yaml(const Landmark & msg, bool use_flow_style = false)
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

}  // namespace cola2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use cola2_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const cola2_msgs::msg::Landmark & msg,
  std::ostream & out, size_t indentation = 0)
{
  cola2_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use cola2_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const cola2_msgs::msg::Landmark & msg)
{
  return cola2_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<cola2_msgs::msg::Landmark>()
{
  return "cola2_msgs::msg::Landmark";
}

template<>
inline const char * name<cola2_msgs::msg::Landmark>()
{
  return "cola2_msgs/msg/Landmark";
}

template<>
struct has_fixed_size<cola2_msgs::msg::Landmark>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<cola2_msgs::msg::Landmark>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<cola2_msgs::msg::Landmark>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // COLA2_MSGS__MSG__DETAIL__LANDMARK__TRAITS_HPP_
