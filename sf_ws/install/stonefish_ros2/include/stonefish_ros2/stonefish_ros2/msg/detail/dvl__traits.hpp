// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from stonefish_ros2:msg/DVL.idl
// generated code does not contain a copyright notice

#ifndef STONEFISH_ROS2__MSG__DETAIL__DVL__TRAITS_HPP_
#define STONEFISH_ROS2__MSG__DETAIL__DVL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "stonefish_ros2/msg/detail/dvl__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'velocity'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"
// Member 'beams'
#include "stonefish_ros2/msg/detail/dvl_beam__traits.hpp"

namespace stonefish_ros2
{

namespace msg
{

inline void to_flow_style_yaml(
  const DVL & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: velocity
  {
    out << "velocity: ";
    to_flow_style_yaml(msg.velocity, out);
    out << ", ";
  }

  // member: velocity_covariance
  {
    if (msg.velocity_covariance.size() == 0) {
      out << "velocity_covariance: []";
    } else {
      out << "velocity_covariance: [";
      size_t pending_items = msg.velocity_covariance.size();
      for (auto item : msg.velocity_covariance) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: altitude
  {
    out << "altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.altitude, out);
    out << ", ";
  }

  // member: beams
  {
    if (msg.beams.size() == 0) {
      out << "beams: []";
    } else {
      out << "beams: [";
      size_t pending_items = msg.beams.size();
      for (auto item : msg.beams) {
        to_flow_style_yaml(item, out);
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
  const DVL & msg,
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

  // member: velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity:\n";
    to_block_style_yaml(msg.velocity, out, indentation + 2);
  }

  // member: velocity_covariance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.velocity_covariance.size() == 0) {
      out << "velocity_covariance: []\n";
    } else {
      out << "velocity_covariance:\n";
      for (auto item : msg.velocity_covariance) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
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

  // member: beams
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.beams.size() == 0) {
      out << "beams: []\n";
    } else {
      out << "beams:\n";
      for (auto item : msg.beams) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DVL & msg, bool use_flow_style = false)
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

}  // namespace stonefish_ros2

namespace rosidl_generator_traits
{

[[deprecated("use stonefish_ros2::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const stonefish_ros2::msg::DVL & msg,
  std::ostream & out, size_t indentation = 0)
{
  stonefish_ros2::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use stonefish_ros2::msg::to_yaml() instead")]]
inline std::string to_yaml(const stonefish_ros2::msg::DVL & msg)
{
  return stonefish_ros2::msg::to_yaml(msg);
}

template<>
inline const char * data_type<stonefish_ros2::msg::DVL>()
{
  return "stonefish_ros2::msg::DVL";
}

template<>
inline const char * name<stonefish_ros2::msg::DVL>()
{
  return "stonefish_ros2/msg/DVL";
}

template<>
struct has_fixed_size<stonefish_ros2::msg::DVL>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<stonefish_ros2::msg::DVL>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<stonefish_ros2::msg::DVL>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // STONEFISH_ROS2__MSG__DETAIL__DVL__TRAITS_HPP_