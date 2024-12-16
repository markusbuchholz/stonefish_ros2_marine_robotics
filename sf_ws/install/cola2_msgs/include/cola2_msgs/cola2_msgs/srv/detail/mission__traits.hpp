// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from cola2_msgs:srv/Mission.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__SRV__DETAIL__MISSION__TRAITS_HPP_
#define COLA2_MSGS__SRV__DETAIL__MISSION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "cola2_msgs/srv/detail/mission__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace cola2_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const Mission_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: mission
  {
    out << "mission: ";
    rosidl_generator_traits::value_to_yaml(msg.mission, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Mission_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: mission
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mission: ";
    rosidl_generator_traits::value_to_yaml(msg.mission, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Mission_Request & msg, bool use_flow_style = false)
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
  const cola2_msgs::srv::Mission_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  cola2_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use cola2_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const cola2_msgs::srv::Mission_Request & msg)
{
  return cola2_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<cola2_msgs::srv::Mission_Request>()
{
  return "cola2_msgs::srv::Mission_Request";
}

template<>
inline const char * name<cola2_msgs::srv::Mission_Request>()
{
  return "cola2_msgs/srv/Mission_Request";
}

template<>
struct has_fixed_size<cola2_msgs::srv::Mission_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<cola2_msgs::srv::Mission_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<cola2_msgs::srv::Mission_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace cola2_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const Mission_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Mission_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Mission_Response & msg, bool use_flow_style = false)
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
  const cola2_msgs::srv::Mission_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  cola2_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use cola2_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const cola2_msgs::srv::Mission_Response & msg)
{
  return cola2_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<cola2_msgs::srv::Mission_Response>()
{
  return "cola2_msgs::srv::Mission_Response";
}

template<>
inline const char * name<cola2_msgs::srv::Mission_Response>()
{
  return "cola2_msgs/srv/Mission_Response";
}

template<>
struct has_fixed_size<cola2_msgs::srv::Mission_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<cola2_msgs::srv::Mission_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<cola2_msgs::srv::Mission_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<cola2_msgs::srv::Mission>()
{
  return "cola2_msgs::srv::Mission";
}

template<>
inline const char * name<cola2_msgs::srv::Mission>()
{
  return "cola2_msgs/srv/Mission";
}

template<>
struct has_fixed_size<cola2_msgs::srv::Mission>
  : std::integral_constant<
    bool,
    has_fixed_size<cola2_msgs::srv::Mission_Request>::value &&
    has_fixed_size<cola2_msgs::srv::Mission_Response>::value
  >
{
};

template<>
struct has_bounded_size<cola2_msgs::srv::Mission>
  : std::integral_constant<
    bool,
    has_bounded_size<cola2_msgs::srv::Mission_Request>::value &&
    has_bounded_size<cola2_msgs::srv::Mission_Response>::value
  >
{
};

template<>
struct is_service<cola2_msgs::srv::Mission>
  : std::true_type
{
};

template<>
struct is_service_request<cola2_msgs::srv::Mission_Request>
  : std::true_type
{
};

template<>
struct is_service_response<cola2_msgs::srv::Mission_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // COLA2_MSGS__SRV__DETAIL__MISSION__TRAITS_HPP_
