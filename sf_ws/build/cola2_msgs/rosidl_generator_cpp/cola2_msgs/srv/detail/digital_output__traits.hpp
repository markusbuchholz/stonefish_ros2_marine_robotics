// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from cola2_msgs:srv/DigitalOutput.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__SRV__DETAIL__DIGITAL_OUTPUT__TRAITS_HPP_
#define COLA2_MSGS__SRV__DETAIL__DIGITAL_OUTPUT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "cola2_msgs/srv/detail/digital_output__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace cola2_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const DigitalOutput_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: digital_output
  {
    out << "digital_output: ";
    rosidl_generator_traits::value_to_yaml(msg.digital_output, out);
    out << ", ";
  }

  // member: value
  {
    out << "value: ";
    rosidl_generator_traits::value_to_yaml(msg.value, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DigitalOutput_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: digital_output
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "digital_output: ";
    rosidl_generator_traits::value_to_yaml(msg.digital_output, out);
    out << "\n";
  }

  // member: value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "value: ";
    rosidl_generator_traits::value_to_yaml(msg.value, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DigitalOutput_Request & msg, bool use_flow_style = false)
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
  const cola2_msgs::srv::DigitalOutput_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  cola2_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use cola2_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const cola2_msgs::srv::DigitalOutput_Request & msg)
{
  return cola2_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<cola2_msgs::srv::DigitalOutput_Request>()
{
  return "cola2_msgs::srv::DigitalOutput_Request";
}

template<>
inline const char * name<cola2_msgs::srv::DigitalOutput_Request>()
{
  return "cola2_msgs/srv/DigitalOutput_Request";
}

template<>
struct has_fixed_size<cola2_msgs::srv::DigitalOutput_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<cola2_msgs::srv::DigitalOutput_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<cola2_msgs::srv::DigitalOutput_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace cola2_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const DigitalOutput_Response & msg,
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
  const DigitalOutput_Response & msg,
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

inline std::string to_yaml(const DigitalOutput_Response & msg, bool use_flow_style = false)
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
  const cola2_msgs::srv::DigitalOutput_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  cola2_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use cola2_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const cola2_msgs::srv::DigitalOutput_Response & msg)
{
  return cola2_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<cola2_msgs::srv::DigitalOutput_Response>()
{
  return "cola2_msgs::srv::DigitalOutput_Response";
}

template<>
inline const char * name<cola2_msgs::srv::DigitalOutput_Response>()
{
  return "cola2_msgs/srv/DigitalOutput_Response";
}

template<>
struct has_fixed_size<cola2_msgs::srv::DigitalOutput_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<cola2_msgs::srv::DigitalOutput_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<cola2_msgs::srv::DigitalOutput_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<cola2_msgs::srv::DigitalOutput>()
{
  return "cola2_msgs::srv::DigitalOutput";
}

template<>
inline const char * name<cola2_msgs::srv::DigitalOutput>()
{
  return "cola2_msgs/srv/DigitalOutput";
}

template<>
struct has_fixed_size<cola2_msgs::srv::DigitalOutput>
  : std::integral_constant<
    bool,
    has_fixed_size<cola2_msgs::srv::DigitalOutput_Request>::value &&
    has_fixed_size<cola2_msgs::srv::DigitalOutput_Response>::value
  >
{
};

template<>
struct has_bounded_size<cola2_msgs::srv::DigitalOutput>
  : std::integral_constant<
    bool,
    has_bounded_size<cola2_msgs::srv::DigitalOutput_Request>::value &&
    has_bounded_size<cola2_msgs::srv::DigitalOutput_Response>::value
  >
{
};

template<>
struct is_service<cola2_msgs::srv::DigitalOutput>
  : std::true_type
{
};

template<>
struct is_service_request<cola2_msgs::srv::DigitalOutput_Request>
  : std::true_type
{
};

template<>
struct is_service_response<cola2_msgs::srv::DigitalOutput_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // COLA2_MSGS__SRV__DETAIL__DIGITAL_OUTPUT__TRAITS_HPP_
