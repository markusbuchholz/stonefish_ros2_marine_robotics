// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from cola2_msgs:msg/NavSts.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__NAV_STS__TRAITS_HPP_
#define COLA2_MSGS__MSG__DETAIL__NAV_STS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "cola2_msgs/msg/detail/nav_sts__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'global_position'
// Member 'origin'
#include "cola2_msgs/msg/detail/decimal_lat_lon__traits.hpp"
// Member 'position'
// Member 'position_variance'
#include "cola2_msgs/msg/detail/ned__traits.hpp"
// Member 'body_velocity'
#include "geometry_msgs/msg/detail/point__traits.hpp"
// Member 'orientation'
// Member 'orientation_rate'
// Member 'orientation_variance'
#include "cola2_msgs/msg/detail/rpy__traits.hpp"

namespace cola2_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const NavSts & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: global_position
  {
    out << "global_position: ";
    to_flow_style_yaml(msg.global_position, out);
    out << ", ";
  }

  // member: origin
  {
    out << "origin: ";
    to_flow_style_yaml(msg.origin, out);
    out << ", ";
  }

  // member: position
  {
    out << "position: ";
    to_flow_style_yaml(msg.position, out);
    out << ", ";
  }

  // member: altitude
  {
    out << "altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.altitude, out);
    out << ", ";
  }

  // member: body_velocity
  {
    out << "body_velocity: ";
    to_flow_style_yaml(msg.body_velocity, out);
    out << ", ";
  }

  // member: orientation
  {
    out << "orientation: ";
    to_flow_style_yaml(msg.orientation, out);
    out << ", ";
  }

  // member: orientation_rate
  {
    out << "orientation_rate: ";
    to_flow_style_yaml(msg.orientation_rate, out);
    out << ", ";
  }

  // member: position_variance
  {
    out << "position_variance: ";
    to_flow_style_yaml(msg.position_variance, out);
    out << ", ";
  }

  // member: orientation_variance
  {
    out << "orientation_variance: ";
    to_flow_style_yaml(msg.orientation_variance, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavSts & msg,
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

  // member: global_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "global_position:\n";
    to_block_style_yaml(msg.global_position, out, indentation + 2);
  }

  // member: origin
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "origin:\n";
    to_block_style_yaml(msg.origin, out, indentation + 2);
  }

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position:\n";
    to_block_style_yaml(msg.position, out, indentation + 2);
  }

  // member: altitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.altitude, out);
    out << "\n";
  }

  // member: body_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "body_velocity:\n";
    to_block_style_yaml(msg.body_velocity, out, indentation + 2);
  }

  // member: orientation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "orientation:\n";
    to_block_style_yaml(msg.orientation, out, indentation + 2);
  }

  // member: orientation_rate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "orientation_rate:\n";
    to_block_style_yaml(msg.orientation_rate, out, indentation + 2);
  }

  // member: position_variance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position_variance:\n";
    to_block_style_yaml(msg.position_variance, out, indentation + 2);
  }

  // member: orientation_variance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "orientation_variance:\n";
    to_block_style_yaml(msg.orientation_variance, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavSts & msg, bool use_flow_style = false)
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
  const cola2_msgs::msg::NavSts & msg,
  std::ostream & out, size_t indentation = 0)
{
  cola2_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use cola2_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const cola2_msgs::msg::NavSts & msg)
{
  return cola2_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<cola2_msgs::msg::NavSts>()
{
  return "cola2_msgs::msg::NavSts";
}

template<>
inline const char * name<cola2_msgs::msg::NavSts>()
{
  return "cola2_msgs/msg/NavSts";
}

template<>
struct has_fixed_size<cola2_msgs::msg::NavSts>
  : std::integral_constant<bool, has_fixed_size<cola2_msgs::msg::DecimalLatLon>::value && has_fixed_size<cola2_msgs::msg::NED>::value && has_fixed_size<cola2_msgs::msg::RPY>::value && has_fixed_size<geometry_msgs::msg::Point>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<cola2_msgs::msg::NavSts>
  : std::integral_constant<bool, has_bounded_size<cola2_msgs::msg::DecimalLatLon>::value && has_bounded_size<cola2_msgs::msg::NED>::value && has_bounded_size<cola2_msgs::msg::RPY>::value && has_bounded_size<geometry_msgs::msg::Point>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<cola2_msgs::msg::NavSts>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // COLA2_MSGS__MSG__DETAIL__NAV_STS__TRAITS_HPP_
