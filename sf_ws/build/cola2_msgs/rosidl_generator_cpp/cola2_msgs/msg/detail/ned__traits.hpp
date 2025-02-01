// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from cola2_msgs:msg/NED.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__NED__TRAITS_HPP_
#define COLA2_MSGS__MSG__DETAIL__NED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "cola2_msgs/msg/detail/ned__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace cola2_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const NED & msg,
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

  // member: depth
  {
    out << "depth: ";
    rosidl_generator_traits::value_to_yaml(msg.depth, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NED & msg,
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

  // member: depth
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "depth: ";
    rosidl_generator_traits::value_to_yaml(msg.depth, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NED & msg, bool use_flow_style = false)
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
  const cola2_msgs::msg::NED & msg,
  std::ostream & out, size_t indentation = 0)
{
  cola2_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use cola2_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const cola2_msgs::msg::NED & msg)
{
  return cola2_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<cola2_msgs::msg::NED>()
{
  return "cola2_msgs::msg::NED";
}

template<>
inline const char * name<cola2_msgs::msg::NED>()
{
  return "cola2_msgs/msg/NED";
}

template<>
struct has_fixed_size<cola2_msgs::msg::NED>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<cola2_msgs::msg::NED>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<cola2_msgs::msg::NED>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // COLA2_MSGS__MSG__DETAIL__NED__TRAITS_HPP_
