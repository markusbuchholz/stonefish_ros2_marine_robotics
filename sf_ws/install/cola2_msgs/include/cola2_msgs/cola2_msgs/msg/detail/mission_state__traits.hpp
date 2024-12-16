// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from cola2_msgs:msg/MissionState.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__MISSION_STATE__TRAITS_HPP_
#define COLA2_MSGS__MSG__DETAIL__MISSION_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "cola2_msgs/msg/detail/mission_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'last_active'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace cola2_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const MissionState & msg,
  std::ostream & out)
{
  out << "{";
  // member: name
  {
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << ", ";
  }

  // member: current_step
  {
    out << "current_step: ";
    rosidl_generator_traits::value_to_yaml(msg.current_step, out);
    out << ", ";
  }

  // member: last_active
  {
    out << "last_active: ";
    to_flow_style_yaml(msg.last_active, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MissionState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << "\n";
  }

  // member: current_step
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_step: ";
    rosidl_generator_traits::value_to_yaml(msg.current_step, out);
    out << "\n";
  }

  // member: last_active
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "last_active:\n";
    to_block_style_yaml(msg.last_active, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MissionState & msg, bool use_flow_style = false)
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
  const cola2_msgs::msg::MissionState & msg,
  std::ostream & out, size_t indentation = 0)
{
  cola2_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use cola2_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const cola2_msgs::msg::MissionState & msg)
{
  return cola2_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<cola2_msgs::msg::MissionState>()
{
  return "cola2_msgs::msg::MissionState";
}

template<>
inline const char * name<cola2_msgs::msg::MissionState>()
{
  return "cola2_msgs/msg/MissionState";
}

template<>
struct has_fixed_size<cola2_msgs::msg::MissionState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<cola2_msgs::msg::MissionState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<cola2_msgs::msg::MissionState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // COLA2_MSGS__MSG__DETAIL__MISSION_STATE__TRAITS_HPP_
