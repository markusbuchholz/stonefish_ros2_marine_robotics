// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from stonefish_ros2:msg/Int32Stamped.idl
// generated code does not contain a copyright notice

#ifndef STONEFISH_ROS2__MSG__DETAIL__INT32_STAMPED__TRAITS_HPP_
#define STONEFISH_ROS2__MSG__DETAIL__INT32_STAMPED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "stonefish_ros2/msg/detail/int32_stamped__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace stonefish_ros2
{

namespace msg
{

inline void to_flow_style_yaml(
  const Int32Stamped & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: data
  {
    out << "data: ";
    rosidl_generator_traits::value_to_yaml(msg.data, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Int32Stamped & msg,
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

  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "data: ";
    rosidl_generator_traits::value_to_yaml(msg.data, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Int32Stamped & msg, bool use_flow_style = false)
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
  const stonefish_ros2::msg::Int32Stamped & msg,
  std::ostream & out, size_t indentation = 0)
{
  stonefish_ros2::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use stonefish_ros2::msg::to_yaml() instead")]]
inline std::string to_yaml(const stonefish_ros2::msg::Int32Stamped & msg)
{
  return stonefish_ros2::msg::to_yaml(msg);
}

template<>
inline const char * data_type<stonefish_ros2::msg::Int32Stamped>()
{
  return "stonefish_ros2::msg::Int32Stamped";
}

template<>
inline const char * name<stonefish_ros2::msg::Int32Stamped>()
{
  return "stonefish_ros2/msg/Int32Stamped";
}

template<>
struct has_fixed_size<stonefish_ros2::msg::Int32Stamped>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<stonefish_ros2::msg::Int32Stamped>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<stonefish_ros2::msg::Int32Stamped>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // STONEFISH_ROS2__MSG__DETAIL__INT32_STAMPED__TRAITS_HPP_
