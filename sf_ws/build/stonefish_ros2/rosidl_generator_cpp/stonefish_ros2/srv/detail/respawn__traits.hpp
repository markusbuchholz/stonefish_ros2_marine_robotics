// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from stonefish_ros2:srv/Respawn.idl
// generated code does not contain a copyright notice

#ifndef STONEFISH_ROS2__SRV__DETAIL__RESPAWN__TRAITS_HPP_
#define STONEFISH_ROS2__SRV__DETAIL__RESPAWN__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "stonefish_ros2/srv/detail/respawn__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'origin'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace stonefish_ros2
{

namespace srv
{

inline void to_flow_style_yaml(
  const Respawn_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: name
  {
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << ", ";
  }

  // member: origin
  {
    out << "origin: ";
    to_flow_style_yaml(msg.origin, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Respawn_Request & msg,
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

  // member: origin
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "origin:\n";
    to_block_style_yaml(msg.origin, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Respawn_Request & msg, bool use_flow_style = false)
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

}  // namespace stonefish_ros2

namespace rosidl_generator_traits
{

[[deprecated("use stonefish_ros2::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const stonefish_ros2::srv::Respawn_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  stonefish_ros2::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use stonefish_ros2::srv::to_yaml() instead")]]
inline std::string to_yaml(const stonefish_ros2::srv::Respawn_Request & msg)
{
  return stonefish_ros2::srv::to_yaml(msg);
}

template<>
inline const char * data_type<stonefish_ros2::srv::Respawn_Request>()
{
  return "stonefish_ros2::srv::Respawn_Request";
}

template<>
inline const char * name<stonefish_ros2::srv::Respawn_Request>()
{
  return "stonefish_ros2/srv/Respawn_Request";
}

template<>
struct has_fixed_size<stonefish_ros2::srv::Respawn_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<stonefish_ros2::srv::Respawn_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<stonefish_ros2::srv::Respawn_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace stonefish_ros2
{

namespace srv
{

inline void to_flow_style_yaml(
  const Respawn_Response & msg,
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
  const Respawn_Response & msg,
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

inline std::string to_yaml(const Respawn_Response & msg, bool use_flow_style = false)
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

}  // namespace stonefish_ros2

namespace rosidl_generator_traits
{

[[deprecated("use stonefish_ros2::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const stonefish_ros2::srv::Respawn_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  stonefish_ros2::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use stonefish_ros2::srv::to_yaml() instead")]]
inline std::string to_yaml(const stonefish_ros2::srv::Respawn_Response & msg)
{
  return stonefish_ros2::srv::to_yaml(msg);
}

template<>
inline const char * data_type<stonefish_ros2::srv::Respawn_Response>()
{
  return "stonefish_ros2::srv::Respawn_Response";
}

template<>
inline const char * name<stonefish_ros2::srv::Respawn_Response>()
{
  return "stonefish_ros2/srv/Respawn_Response";
}

template<>
struct has_fixed_size<stonefish_ros2::srv::Respawn_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<stonefish_ros2::srv::Respawn_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<stonefish_ros2::srv::Respawn_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<stonefish_ros2::srv::Respawn>()
{
  return "stonefish_ros2::srv::Respawn";
}

template<>
inline const char * name<stonefish_ros2::srv::Respawn>()
{
  return "stonefish_ros2/srv/Respawn";
}

template<>
struct has_fixed_size<stonefish_ros2::srv::Respawn>
  : std::integral_constant<
    bool,
    has_fixed_size<stonefish_ros2::srv::Respawn_Request>::value &&
    has_fixed_size<stonefish_ros2::srv::Respawn_Response>::value
  >
{
};

template<>
struct has_bounded_size<stonefish_ros2::srv::Respawn>
  : std::integral_constant<
    bool,
    has_bounded_size<stonefish_ros2::srv::Respawn_Request>::value &&
    has_bounded_size<stonefish_ros2::srv::Respawn_Response>::value
  >
{
};

template<>
struct is_service<stonefish_ros2::srv::Respawn>
  : std::true_type
{
};

template<>
struct is_service_request<stonefish_ros2::srv::Respawn_Request>
  : std::true_type
{
};

template<>
struct is_service_response<stonefish_ros2::srv::Respawn_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // STONEFISH_ROS2__SRV__DETAIL__RESPAWN__TRAITS_HPP_
