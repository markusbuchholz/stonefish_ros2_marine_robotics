// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from cola2_msgs:msg/BodyForceReq.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__BODY_FORCE_REQ__TRAITS_HPP_
#define COLA2_MSGS__MSG__DETAIL__BODY_FORCE_REQ__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "cola2_msgs/msg/detail/body_force_req__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'goal'
#include "cola2_msgs/msg/detail/goal_descriptor__traits.hpp"
// Member 'wrench'
#include "geometry_msgs/msg/detail/wrench__traits.hpp"
// Member 'disable_axis'
#include "cola2_msgs/msg/detail/bool6_axis__traits.hpp"

namespace cola2_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const BodyForceReq & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
    out << ", ";
  }

  // member: wrench
  {
    out << "wrench: ";
    to_flow_style_yaml(msg.wrench, out);
    out << ", ";
  }

  // member: disable_axis
  {
    out << "disable_axis: ";
    to_flow_style_yaml(msg.disable_axis, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BodyForceReq & msg,
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

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }

  // member: wrench
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wrench:\n";
    to_block_style_yaml(msg.wrench, out, indentation + 2);
  }

  // member: disable_axis
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "disable_axis:\n";
    to_block_style_yaml(msg.disable_axis, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BodyForceReq & msg, bool use_flow_style = false)
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
  const cola2_msgs::msg::BodyForceReq & msg,
  std::ostream & out, size_t indentation = 0)
{
  cola2_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use cola2_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const cola2_msgs::msg::BodyForceReq & msg)
{
  return cola2_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<cola2_msgs::msg::BodyForceReq>()
{
  return "cola2_msgs::msg::BodyForceReq";
}

template<>
inline const char * name<cola2_msgs::msg::BodyForceReq>()
{
  return "cola2_msgs/msg/BodyForceReq";
}

template<>
struct has_fixed_size<cola2_msgs::msg::BodyForceReq>
  : std::integral_constant<bool, has_fixed_size<cola2_msgs::msg::Bool6Axis>::value && has_fixed_size<cola2_msgs::msg::GoalDescriptor>::value && has_fixed_size<geometry_msgs::msg::Wrench>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<cola2_msgs::msg::BodyForceReq>
  : std::integral_constant<bool, has_bounded_size<cola2_msgs::msg::Bool6Axis>::value && has_bounded_size<cola2_msgs::msg::GoalDescriptor>::value && has_bounded_size<geometry_msgs::msg::Wrench>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<cola2_msgs::msg::BodyForceReq>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // COLA2_MSGS__MSG__DETAIL__BODY_FORCE_REQ__TRAITS_HPP_
