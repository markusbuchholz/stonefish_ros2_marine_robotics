// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from cola2_msgs:msg/GoalDescriptor.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__GOAL_DESCRIPTOR__TRAITS_HPP_
#define COLA2_MSGS__MSG__DETAIL__GOAL_DESCRIPTOR__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "cola2_msgs/msg/detail/goal_descriptor__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace cola2_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const GoalDescriptor & msg,
  std::ostream & out)
{
  out << "{";
  // member: requester
  {
    out << "requester: ";
    rosidl_generator_traits::value_to_yaml(msg.requester, out);
    out << ", ";
  }

  // member: priority
  {
    out << "priority: ";
    rosidl_generator_traits::value_to_yaml(msg.priority, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GoalDescriptor & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: requester
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "requester: ";
    rosidl_generator_traits::value_to_yaml(msg.requester, out);
    out << "\n";
  }

  // member: priority
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "priority: ";
    rosidl_generator_traits::value_to_yaml(msg.priority, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GoalDescriptor & msg, bool use_flow_style = false)
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
  const cola2_msgs::msg::GoalDescriptor & msg,
  std::ostream & out, size_t indentation = 0)
{
  cola2_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use cola2_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const cola2_msgs::msg::GoalDescriptor & msg)
{
  return cola2_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<cola2_msgs::msg::GoalDescriptor>()
{
  return "cola2_msgs::msg::GoalDescriptor";
}

template<>
inline const char * name<cola2_msgs::msg::GoalDescriptor>()
{
  return "cola2_msgs/msg/GoalDescriptor";
}

template<>
struct has_fixed_size<cola2_msgs::msg::GoalDescriptor>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<cola2_msgs::msg::GoalDescriptor>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<cola2_msgs::msg::GoalDescriptor>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // COLA2_MSGS__MSG__DETAIL__GOAL_DESCRIPTOR__TRAITS_HPP_
