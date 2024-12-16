// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from cola2_msgs:msg/SafetySupervisorStatus.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__SAFETY_SUPERVISOR_STATUS__TRAITS_HPP_
#define COLA2_MSGS__MSG__DETAIL__SAFETY_SUPERVISOR_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "cola2_msgs/msg/detail/safety_supervisor_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'recovery_action'
#include "cola2_msgs/msg/detail/recovery_action__traits.hpp"

namespace cola2_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const SafetySupervisorStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: status_code
  {
    out << "status_code: ";
    rosidl_generator_traits::value_to_yaml(msg.status_code, out);
    out << ", ";
  }

  // member: recovery_action
  {
    out << "recovery_action: ";
    to_flow_style_yaml(msg.recovery_action, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SafetySupervisorStatus & msg,
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

  // member: status_code
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status_code: ";
    rosidl_generator_traits::value_to_yaml(msg.status_code, out);
    out << "\n";
  }

  // member: recovery_action
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "recovery_action:\n";
    to_block_style_yaml(msg.recovery_action, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SafetySupervisorStatus & msg, bool use_flow_style = false)
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
  const cola2_msgs::msg::SafetySupervisorStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  cola2_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use cola2_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const cola2_msgs::msg::SafetySupervisorStatus & msg)
{
  return cola2_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<cola2_msgs::msg::SafetySupervisorStatus>()
{
  return "cola2_msgs::msg::SafetySupervisorStatus";
}

template<>
inline const char * name<cola2_msgs::msg::SafetySupervisorStatus>()
{
  return "cola2_msgs/msg/SafetySupervisorStatus";
}

template<>
struct has_fixed_size<cola2_msgs::msg::SafetySupervisorStatus>
  : std::integral_constant<bool, has_fixed_size<cola2_msgs::msg::RecoveryAction>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<cola2_msgs::msg::SafetySupervisorStatus>
  : std::integral_constant<bool, has_bounded_size<cola2_msgs::msg::RecoveryAction>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<cola2_msgs::msg::SafetySupervisorStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // COLA2_MSGS__MSG__DETAIL__SAFETY_SUPERVISOR_STATUS__TRAITS_HPP_
