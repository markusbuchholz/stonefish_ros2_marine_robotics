// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from cola2_msgs:srv/Goto.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__SRV__DETAIL__GOTO__TRAITS_HPP_
#define COLA2_MSGS__SRV__DETAIL__GOTO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "cola2_msgs/srv/detail/goto__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace cola2_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const Goto_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: final_x
  {
    out << "final_x: ";
    rosidl_generator_traits::value_to_yaml(msg.final_x, out);
    out << ", ";
  }

  // member: final_y
  {
    out << "final_y: ";
    rosidl_generator_traits::value_to_yaml(msg.final_y, out);
    out << ", ";
  }

  // member: final_depth
  {
    out << "final_depth: ";
    rosidl_generator_traits::value_to_yaml(msg.final_depth, out);
    out << ", ";
  }

  // member: final_altitude
  {
    out << "final_altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.final_altitude, out);
    out << ", ";
  }

  // member: reference
  {
    out << "reference: ";
    rosidl_generator_traits::value_to_yaml(msg.reference, out);
    out << ", ";
  }

  // member: heave_mode
  {
    out << "heave_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.heave_mode, out);
    out << ", ";
  }

  // member: surge_velocity
  {
    out << "surge_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.surge_velocity, out);
    out << ", ";
  }

  // member: tolerance_xy
  {
    out << "tolerance_xy: ";
    rosidl_generator_traits::value_to_yaml(msg.tolerance_xy, out);
    out << ", ";
  }

  // member: timeout
  {
    out << "timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.timeout, out);
    out << ", ";
  }

  // member: no_altitude_goes_up
  {
    out << "no_altitude_goes_up: ";
    rosidl_generator_traits::value_to_yaml(msg.no_altitude_goes_up, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Goto_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: final_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "final_x: ";
    rosidl_generator_traits::value_to_yaml(msg.final_x, out);
    out << "\n";
  }

  // member: final_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "final_y: ";
    rosidl_generator_traits::value_to_yaml(msg.final_y, out);
    out << "\n";
  }

  // member: final_depth
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "final_depth: ";
    rosidl_generator_traits::value_to_yaml(msg.final_depth, out);
    out << "\n";
  }

  // member: final_altitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "final_altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.final_altitude, out);
    out << "\n";
  }

  // member: reference
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reference: ";
    rosidl_generator_traits::value_to_yaml(msg.reference, out);
    out << "\n";
  }

  // member: heave_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "heave_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.heave_mode, out);
    out << "\n";
  }

  // member: surge_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "surge_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.surge_velocity, out);
    out << "\n";
  }

  // member: tolerance_xy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tolerance_xy: ";
    rosidl_generator_traits::value_to_yaml(msg.tolerance_xy, out);
    out << "\n";
  }

  // member: timeout
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.timeout, out);
    out << "\n";
  }

  // member: no_altitude_goes_up
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "no_altitude_goes_up: ";
    rosidl_generator_traits::value_to_yaml(msg.no_altitude_goes_up, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Goto_Request & msg, bool use_flow_style = false)
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
  const cola2_msgs::srv::Goto_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  cola2_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use cola2_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const cola2_msgs::srv::Goto_Request & msg)
{
  return cola2_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<cola2_msgs::srv::Goto_Request>()
{
  return "cola2_msgs::srv::Goto_Request";
}

template<>
inline const char * name<cola2_msgs::srv::Goto_Request>()
{
  return "cola2_msgs/srv/Goto_Request";
}

template<>
struct has_fixed_size<cola2_msgs::srv::Goto_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<cola2_msgs::srv::Goto_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<cola2_msgs::srv::Goto_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace cola2_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const Goto_Response & msg,
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
  const Goto_Response & msg,
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

inline std::string to_yaml(const Goto_Response & msg, bool use_flow_style = false)
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
  const cola2_msgs::srv::Goto_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  cola2_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use cola2_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const cola2_msgs::srv::Goto_Response & msg)
{
  return cola2_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<cola2_msgs::srv::Goto_Response>()
{
  return "cola2_msgs::srv::Goto_Response";
}

template<>
inline const char * name<cola2_msgs::srv::Goto_Response>()
{
  return "cola2_msgs/srv/Goto_Response";
}

template<>
struct has_fixed_size<cola2_msgs::srv::Goto_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<cola2_msgs::srv::Goto_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<cola2_msgs::srv::Goto_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<cola2_msgs::srv::Goto>()
{
  return "cola2_msgs::srv::Goto";
}

template<>
inline const char * name<cola2_msgs::srv::Goto>()
{
  return "cola2_msgs/srv/Goto";
}

template<>
struct has_fixed_size<cola2_msgs::srv::Goto>
  : std::integral_constant<
    bool,
    has_fixed_size<cola2_msgs::srv::Goto_Request>::value &&
    has_fixed_size<cola2_msgs::srv::Goto_Response>::value
  >
{
};

template<>
struct has_bounded_size<cola2_msgs::srv::Goto>
  : std::integral_constant<
    bool,
    has_bounded_size<cola2_msgs::srv::Goto_Request>::value &&
    has_bounded_size<cola2_msgs::srv::Goto_Response>::value
  >
{
};

template<>
struct is_service<cola2_msgs::srv::Goto>
  : std::true_type
{
};

template<>
struct is_service_request<cola2_msgs::srv::Goto_Request>
  : std::true_type
{
};

template<>
struct is_service_response<cola2_msgs::srv::Goto_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // COLA2_MSGS__SRV__DETAIL__GOTO__TRAITS_HPP_
