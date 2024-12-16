// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from stonefish_ros2:msg/DebugPhysics.idl
// generated code does not contain a copyright notice

#ifndef STONEFISH_ROS2__MSG__DETAIL__DEBUG_PHYSICS__TRAITS_HPP_
#define STONEFISH_ROS2__MSG__DETAIL__DEBUG_PHYSICS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "stonefish_ros2/msg/detail/debug_physics__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'cog'
// Member 'cob'
// Member 'inertia'
// Member 'damping_coeff'
// Member 'skin_friction_coeff'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"
// Member 'velocity'
#include "geometry_msgs/msg/detail/twist__traits.hpp"
// Member 'buoyancy'
// Member 'damping'
// Member 'skin_friction'
#include "geometry_msgs/msg/detail/wrench__traits.hpp"

namespace stonefish_ros2
{

namespace msg
{

inline void to_flow_style_yaml(
  const DebugPhysics & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: mass
  {
    out << "mass: ";
    rosidl_generator_traits::value_to_yaml(msg.mass, out);
    out << ", ";
  }

  // member: volume
  {
    out << "volume: ";
    rosidl_generator_traits::value_to_yaml(msg.volume, out);
    out << ", ";
  }

  // member: surface
  {
    out << "surface: ";
    rosidl_generator_traits::value_to_yaml(msg.surface, out);
    out << ", ";
  }

  // member: cog
  {
    out << "cog: ";
    to_flow_style_yaml(msg.cog, out);
    out << ", ";
  }

  // member: cob
  {
    out << "cob: ";
    to_flow_style_yaml(msg.cob, out);
    out << ", ";
  }

  // member: inertia
  {
    out << "inertia: ";
    to_flow_style_yaml(msg.inertia, out);
    out << ", ";
  }

  // member: damping_coeff
  {
    out << "damping_coeff: ";
    to_flow_style_yaml(msg.damping_coeff, out);
    out << ", ";
  }

  // member: skin_friction_coeff
  {
    out << "skin_friction_coeff: ";
    to_flow_style_yaml(msg.skin_friction_coeff, out);
    out << ", ";
  }

  // member: velocity
  {
    out << "velocity: ";
    to_flow_style_yaml(msg.velocity, out);
    out << ", ";
  }

  // member: buoyancy
  {
    out << "buoyancy: ";
    to_flow_style_yaml(msg.buoyancy, out);
    out << ", ";
  }

  // member: damping
  {
    out << "damping: ";
    to_flow_style_yaml(msg.damping, out);
    out << ", ";
  }

  // member: skin_friction
  {
    out << "skin_friction: ";
    to_flow_style_yaml(msg.skin_friction, out);
    out << ", ";
  }

  // member: wetted_surface
  {
    out << "wetted_surface: ";
    rosidl_generator_traits::value_to_yaml(msg.wetted_surface, out);
    out << ", ";
  }

  // member: submerged_volume
  {
    out << "submerged_volume: ";
    rosidl_generator_traits::value_to_yaml(msg.submerged_volume, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DebugPhysics & msg,
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

  // member: mass
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mass: ";
    rosidl_generator_traits::value_to_yaml(msg.mass, out);
    out << "\n";
  }

  // member: volume
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "volume: ";
    rosidl_generator_traits::value_to_yaml(msg.volume, out);
    out << "\n";
  }

  // member: surface
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "surface: ";
    rosidl_generator_traits::value_to_yaml(msg.surface, out);
    out << "\n";
  }

  // member: cog
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cog:\n";
    to_block_style_yaml(msg.cog, out, indentation + 2);
  }

  // member: cob
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cob:\n";
    to_block_style_yaml(msg.cob, out, indentation + 2);
  }

  // member: inertia
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "inertia:\n";
    to_block_style_yaml(msg.inertia, out, indentation + 2);
  }

  // member: damping_coeff
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "damping_coeff:\n";
    to_block_style_yaml(msg.damping_coeff, out, indentation + 2);
  }

  // member: skin_friction_coeff
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "skin_friction_coeff:\n";
    to_block_style_yaml(msg.skin_friction_coeff, out, indentation + 2);
  }

  // member: velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity:\n";
    to_block_style_yaml(msg.velocity, out, indentation + 2);
  }

  // member: buoyancy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "buoyancy:\n";
    to_block_style_yaml(msg.buoyancy, out, indentation + 2);
  }

  // member: damping
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "damping:\n";
    to_block_style_yaml(msg.damping, out, indentation + 2);
  }

  // member: skin_friction
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "skin_friction:\n";
    to_block_style_yaml(msg.skin_friction, out, indentation + 2);
  }

  // member: wetted_surface
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wetted_surface: ";
    rosidl_generator_traits::value_to_yaml(msg.wetted_surface, out);
    out << "\n";
  }

  // member: submerged_volume
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "submerged_volume: ";
    rosidl_generator_traits::value_to_yaml(msg.submerged_volume, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DebugPhysics & msg, bool use_flow_style = false)
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
  const stonefish_ros2::msg::DebugPhysics & msg,
  std::ostream & out, size_t indentation = 0)
{
  stonefish_ros2::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use stonefish_ros2::msg::to_yaml() instead")]]
inline std::string to_yaml(const stonefish_ros2::msg::DebugPhysics & msg)
{
  return stonefish_ros2::msg::to_yaml(msg);
}

template<>
inline const char * data_type<stonefish_ros2::msg::DebugPhysics>()
{
  return "stonefish_ros2::msg::DebugPhysics";
}

template<>
inline const char * name<stonefish_ros2::msg::DebugPhysics>()
{
  return "stonefish_ros2/msg/DebugPhysics";
}

template<>
struct has_fixed_size<stonefish_ros2::msg::DebugPhysics>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Twist>::value && has_fixed_size<geometry_msgs::msg::Vector3>::value && has_fixed_size<geometry_msgs::msg::Wrench>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<stonefish_ros2::msg::DebugPhysics>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Twist>::value && has_bounded_size<geometry_msgs::msg::Vector3>::value && has_bounded_size<geometry_msgs::msg::Wrench>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<stonefish_ros2::msg::DebugPhysics>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // STONEFISH_ROS2__MSG__DETAIL__DEBUG_PHYSICS__TRAITS_HPP_
