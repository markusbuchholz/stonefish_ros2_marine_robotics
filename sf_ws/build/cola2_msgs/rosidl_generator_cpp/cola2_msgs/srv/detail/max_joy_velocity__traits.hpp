// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from cola2_msgs:srv/MaxJoyVelocity.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__SRV__DETAIL__MAX_JOY_VELOCITY__TRAITS_HPP_
#define COLA2_MSGS__SRV__DETAIL__MAX_JOY_VELOCITY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "cola2_msgs/srv/detail/max_joy_velocity__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace cola2_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const MaxJoyVelocity_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: max_joy_velocity
  {
    if (msg.max_joy_velocity.size() == 0) {
      out << "max_joy_velocity: []";
    } else {
      out << "max_joy_velocity: [";
      size_t pending_items = msg.max_joy_velocity.size();
      for (auto item : msg.max_joy_velocity) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MaxJoyVelocity_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: max_joy_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.max_joy_velocity.size() == 0) {
      out << "max_joy_velocity: []\n";
    } else {
      out << "max_joy_velocity:\n";
      for (auto item : msg.max_joy_velocity) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MaxJoyVelocity_Request & msg, bool use_flow_style = false)
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
  const cola2_msgs::srv::MaxJoyVelocity_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  cola2_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use cola2_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const cola2_msgs::srv::MaxJoyVelocity_Request & msg)
{
  return cola2_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<cola2_msgs::srv::MaxJoyVelocity_Request>()
{
  return "cola2_msgs::srv::MaxJoyVelocity_Request";
}

template<>
inline const char * name<cola2_msgs::srv::MaxJoyVelocity_Request>()
{
  return "cola2_msgs/srv/MaxJoyVelocity_Request";
}

template<>
struct has_fixed_size<cola2_msgs::srv::MaxJoyVelocity_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<cola2_msgs::srv::MaxJoyVelocity_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<cola2_msgs::srv::MaxJoyVelocity_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace cola2_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const MaxJoyVelocity_Response & msg,
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
  const MaxJoyVelocity_Response & msg,
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

inline std::string to_yaml(const MaxJoyVelocity_Response & msg, bool use_flow_style = false)
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
  const cola2_msgs::srv::MaxJoyVelocity_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  cola2_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use cola2_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const cola2_msgs::srv::MaxJoyVelocity_Response & msg)
{
  return cola2_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<cola2_msgs::srv::MaxJoyVelocity_Response>()
{
  return "cola2_msgs::srv::MaxJoyVelocity_Response";
}

template<>
inline const char * name<cola2_msgs::srv::MaxJoyVelocity_Response>()
{
  return "cola2_msgs/srv/MaxJoyVelocity_Response";
}

template<>
struct has_fixed_size<cola2_msgs::srv::MaxJoyVelocity_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<cola2_msgs::srv::MaxJoyVelocity_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<cola2_msgs::srv::MaxJoyVelocity_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<cola2_msgs::srv::MaxJoyVelocity>()
{
  return "cola2_msgs::srv::MaxJoyVelocity";
}

template<>
inline const char * name<cola2_msgs::srv::MaxJoyVelocity>()
{
  return "cola2_msgs/srv/MaxJoyVelocity";
}

template<>
struct has_fixed_size<cola2_msgs::srv::MaxJoyVelocity>
  : std::integral_constant<
    bool,
    has_fixed_size<cola2_msgs::srv::MaxJoyVelocity_Request>::value &&
    has_fixed_size<cola2_msgs::srv::MaxJoyVelocity_Response>::value
  >
{
};

template<>
struct has_bounded_size<cola2_msgs::srv::MaxJoyVelocity>
  : std::integral_constant<
    bool,
    has_bounded_size<cola2_msgs::srv::MaxJoyVelocity_Request>::value &&
    has_bounded_size<cola2_msgs::srv::MaxJoyVelocity_Response>::value
  >
{
};

template<>
struct is_service<cola2_msgs::srv::MaxJoyVelocity>
  : std::true_type
{
};

template<>
struct is_service_request<cola2_msgs::srv::MaxJoyVelocity_Request>
  : std::true_type
{
};

template<>
struct is_service_response<cola2_msgs::srv::MaxJoyVelocity_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // COLA2_MSGS__SRV__DETAIL__MAX_JOY_VELOCITY__TRAITS_HPP_
