// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from stonefish_ros2:srv/SonarSettings2.idl
// generated code does not contain a copyright notice

#ifndef STONEFISH_ROS2__SRV__DETAIL__SONAR_SETTINGS2__TRAITS_HPP_
#define STONEFISH_ROS2__SRV__DETAIL__SONAR_SETTINGS2__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "stonefish_ros2/srv/detail/sonar_settings2__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace stonefish_ros2
{

namespace srv
{

inline void to_flow_style_yaml(
  const SonarSettings2_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: range_min
  {
    out << "range_min: ";
    rosidl_generator_traits::value_to_yaml(msg.range_min, out);
    out << ", ";
  }

  // member: range_max
  {
    out << "range_max: ";
    rosidl_generator_traits::value_to_yaml(msg.range_max, out);
    out << ", ";
  }

  // member: rotation_min
  {
    out << "rotation_min: ";
    rosidl_generator_traits::value_to_yaml(msg.rotation_min, out);
    out << ", ";
  }

  // member: rotation_max
  {
    out << "rotation_max: ";
    rosidl_generator_traits::value_to_yaml(msg.rotation_max, out);
    out << ", ";
  }

  // member: gain
  {
    out << "gain: ";
    rosidl_generator_traits::value_to_yaml(msg.gain, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SonarSettings2_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: range_min
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "range_min: ";
    rosidl_generator_traits::value_to_yaml(msg.range_min, out);
    out << "\n";
  }

  // member: range_max
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "range_max: ";
    rosidl_generator_traits::value_to_yaml(msg.range_max, out);
    out << "\n";
  }

  // member: rotation_min
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rotation_min: ";
    rosidl_generator_traits::value_to_yaml(msg.rotation_min, out);
    out << "\n";
  }

  // member: rotation_max
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rotation_max: ";
    rosidl_generator_traits::value_to_yaml(msg.rotation_max, out);
    out << "\n";
  }

  // member: gain
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gain: ";
    rosidl_generator_traits::value_to_yaml(msg.gain, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SonarSettings2_Request & msg, bool use_flow_style = false)
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
  const stonefish_ros2::srv::SonarSettings2_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  stonefish_ros2::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use stonefish_ros2::srv::to_yaml() instead")]]
inline std::string to_yaml(const stonefish_ros2::srv::SonarSettings2_Request & msg)
{
  return stonefish_ros2::srv::to_yaml(msg);
}

template<>
inline const char * data_type<stonefish_ros2::srv::SonarSettings2_Request>()
{
  return "stonefish_ros2::srv::SonarSettings2_Request";
}

template<>
inline const char * name<stonefish_ros2::srv::SonarSettings2_Request>()
{
  return "stonefish_ros2/srv/SonarSettings2_Request";
}

template<>
struct has_fixed_size<stonefish_ros2::srv::SonarSettings2_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<stonefish_ros2::srv::SonarSettings2_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<stonefish_ros2::srv::SonarSettings2_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace stonefish_ros2
{

namespace srv
{

inline void to_flow_style_yaml(
  const SonarSettings2_Response & msg,
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
  const SonarSettings2_Response & msg,
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

inline std::string to_yaml(const SonarSettings2_Response & msg, bool use_flow_style = false)
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
  const stonefish_ros2::srv::SonarSettings2_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  stonefish_ros2::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use stonefish_ros2::srv::to_yaml() instead")]]
inline std::string to_yaml(const stonefish_ros2::srv::SonarSettings2_Response & msg)
{
  return stonefish_ros2::srv::to_yaml(msg);
}

template<>
inline const char * data_type<stonefish_ros2::srv::SonarSettings2_Response>()
{
  return "stonefish_ros2::srv::SonarSettings2_Response";
}

template<>
inline const char * name<stonefish_ros2::srv::SonarSettings2_Response>()
{
  return "stonefish_ros2/srv/SonarSettings2_Response";
}

template<>
struct has_fixed_size<stonefish_ros2::srv::SonarSettings2_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<stonefish_ros2::srv::SonarSettings2_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<stonefish_ros2::srv::SonarSettings2_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<stonefish_ros2::srv::SonarSettings2>()
{
  return "stonefish_ros2::srv::SonarSettings2";
}

template<>
inline const char * name<stonefish_ros2::srv::SonarSettings2>()
{
  return "stonefish_ros2/srv/SonarSettings2";
}

template<>
struct has_fixed_size<stonefish_ros2::srv::SonarSettings2>
  : std::integral_constant<
    bool,
    has_fixed_size<stonefish_ros2::srv::SonarSettings2_Request>::value &&
    has_fixed_size<stonefish_ros2::srv::SonarSettings2_Response>::value
  >
{
};

template<>
struct has_bounded_size<stonefish_ros2::srv::SonarSettings2>
  : std::integral_constant<
    bool,
    has_bounded_size<stonefish_ros2::srv::SonarSettings2_Request>::value &&
    has_bounded_size<stonefish_ros2::srv::SonarSettings2_Response>::value
  >
{
};

template<>
struct is_service<stonefish_ros2::srv::SonarSettings2>
  : std::true_type
{
};

template<>
struct is_service_request<stonefish_ros2::srv::SonarSettings2_Request>
  : std::true_type
{
};

template<>
struct is_service_response<stonefish_ros2::srv::SonarSettings2_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // STONEFISH_ROS2__SRV__DETAIL__SONAR_SETTINGS2__TRAITS_HPP_
