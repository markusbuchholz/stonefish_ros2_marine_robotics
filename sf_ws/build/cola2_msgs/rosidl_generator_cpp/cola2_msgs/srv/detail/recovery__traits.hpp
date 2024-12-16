// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from cola2_msgs:srv/Recovery.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__SRV__DETAIL__RECOVERY__TRAITS_HPP_
#define COLA2_MSGS__SRV__DETAIL__RECOVERY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "cola2_msgs/srv/detail/recovery__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'requested_action'
#include "cola2_msgs/msg/detail/recovery_action__traits.hpp"

namespace cola2_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const Recovery_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: requested_action
  {
    out << "requested_action: ";
    to_flow_style_yaml(msg.requested_action, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Recovery_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: requested_action
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "requested_action:\n";
    to_block_style_yaml(msg.requested_action, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Recovery_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace cola2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use cola2_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const cola2_msgs::srv::Recovery_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  cola2_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use cola2_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const cola2_msgs::srv::Recovery_Request & msg)
{
  return cola2_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<cola2_msgs::srv::Recovery_Request>()
{
  return "cola2_msgs::srv::Recovery_Request";
}

template<>
inline const char * name<cola2_msgs::srv::Recovery_Request>()
{
  return "cola2_msgs/srv/Recovery_Request";
}

template<>
struct has_fixed_size<cola2_msgs::srv::Recovery_Request>
  : std::integral_constant<bool, has_fixed_size<cola2_msgs::msg::RecoveryAction>::value> {};

template<>
struct has_bounded_size<cola2_msgs::srv::Recovery_Request>
  : std::integral_constant<bool, has_bounded_size<cola2_msgs::msg::RecoveryAction>::value> {};

template<>
struct is_message<cola2_msgs::srv::Recovery_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace cola2_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const Recovery_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: attempted
  {
    out << "attempted: ";
    rosidl_generator_traits::value_to_yaml(msg.attempted, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Recovery_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: attempted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "attempted: ";
    rosidl_generator_traits::value_to_yaml(msg.attempted, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Recovery_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace cola2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use cola2_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const cola2_msgs::srv::Recovery_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  cola2_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use cola2_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const cola2_msgs::srv::Recovery_Response & msg)
{
  return cola2_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<cola2_msgs::srv::Recovery_Response>()
{
  return "cola2_msgs::srv::Recovery_Response";
}

template<>
inline const char * name<cola2_msgs::srv::Recovery_Response>()
{
  return "cola2_msgs/srv/Recovery_Response";
}

template<>
struct has_fixed_size<cola2_msgs::srv::Recovery_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<cola2_msgs::srv::Recovery_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<cola2_msgs::srv::Recovery_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<cola2_msgs::srv::Recovery>()
{
  return "cola2_msgs::srv::Recovery";
}

template<>
inline const char * name<cola2_msgs::srv::Recovery>()
{
  return "cola2_msgs/srv/Recovery";
}

template<>
struct has_fixed_size<cola2_msgs::srv::Recovery>
  : std::integral_constant<
    bool,
    has_fixed_size<cola2_msgs::srv::Recovery_Request>::value &&
    has_fixed_size<cola2_msgs::srv::Recovery_Response>::value
  >
{
};

template<>
struct has_bounded_size<cola2_msgs::srv::Recovery>
  : std::integral_constant<
    bool,
    has_bounded_size<cola2_msgs::srv::Recovery_Request>::value &&
    has_bounded_size<cola2_msgs::srv::Recovery_Response>::value
  >
{
};

template<>
struct is_service<cola2_msgs::srv::Recovery>
  : std::true_type
{
};

template<>
struct is_service_request<cola2_msgs::srv::Recovery_Request>
  : std::true_type
{
};

template<>
struct is_service_response<cola2_msgs::srv::Recovery_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // COLA2_MSGS__SRV__DETAIL__RECOVERY__TRAITS_HPP_
