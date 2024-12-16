// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from stonefish_ros2:msg/DebugPhysics.idl
// generated code does not contain a copyright notice

#ifndef STONEFISH_ROS2__MSG__DETAIL__DEBUG_PHYSICS__BUILDER_HPP_
#define STONEFISH_ROS2__MSG__DETAIL__DEBUG_PHYSICS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "stonefish_ros2/msg/detail/debug_physics__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace stonefish_ros2
{

namespace msg
{

namespace builder
{

class Init_DebugPhysics_submerged_volume
{
public:
  explicit Init_DebugPhysics_submerged_volume(::stonefish_ros2::msg::DebugPhysics & msg)
  : msg_(msg)
  {}
  ::stonefish_ros2::msg::DebugPhysics submerged_volume(::stonefish_ros2::msg::DebugPhysics::_submerged_volume_type arg)
  {
    msg_.submerged_volume = std::move(arg);
    return std::move(msg_);
  }

private:
  ::stonefish_ros2::msg::DebugPhysics msg_;
};

class Init_DebugPhysics_wetted_surface
{
public:
  explicit Init_DebugPhysics_wetted_surface(::stonefish_ros2::msg::DebugPhysics & msg)
  : msg_(msg)
  {}
  Init_DebugPhysics_submerged_volume wetted_surface(::stonefish_ros2::msg::DebugPhysics::_wetted_surface_type arg)
  {
    msg_.wetted_surface = std::move(arg);
    return Init_DebugPhysics_submerged_volume(msg_);
  }

private:
  ::stonefish_ros2::msg::DebugPhysics msg_;
};

class Init_DebugPhysics_skin_friction
{
public:
  explicit Init_DebugPhysics_skin_friction(::stonefish_ros2::msg::DebugPhysics & msg)
  : msg_(msg)
  {}
  Init_DebugPhysics_wetted_surface skin_friction(::stonefish_ros2::msg::DebugPhysics::_skin_friction_type arg)
  {
    msg_.skin_friction = std::move(arg);
    return Init_DebugPhysics_wetted_surface(msg_);
  }

private:
  ::stonefish_ros2::msg::DebugPhysics msg_;
};

class Init_DebugPhysics_damping
{
public:
  explicit Init_DebugPhysics_damping(::stonefish_ros2::msg::DebugPhysics & msg)
  : msg_(msg)
  {}
  Init_DebugPhysics_skin_friction damping(::stonefish_ros2::msg::DebugPhysics::_damping_type arg)
  {
    msg_.damping = std::move(arg);
    return Init_DebugPhysics_skin_friction(msg_);
  }

private:
  ::stonefish_ros2::msg::DebugPhysics msg_;
};

class Init_DebugPhysics_buoyancy
{
public:
  explicit Init_DebugPhysics_buoyancy(::stonefish_ros2::msg::DebugPhysics & msg)
  : msg_(msg)
  {}
  Init_DebugPhysics_damping buoyancy(::stonefish_ros2::msg::DebugPhysics::_buoyancy_type arg)
  {
    msg_.buoyancy = std::move(arg);
    return Init_DebugPhysics_damping(msg_);
  }

private:
  ::stonefish_ros2::msg::DebugPhysics msg_;
};

class Init_DebugPhysics_velocity
{
public:
  explicit Init_DebugPhysics_velocity(::stonefish_ros2::msg::DebugPhysics & msg)
  : msg_(msg)
  {}
  Init_DebugPhysics_buoyancy velocity(::stonefish_ros2::msg::DebugPhysics::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_DebugPhysics_buoyancy(msg_);
  }

private:
  ::stonefish_ros2::msg::DebugPhysics msg_;
};

class Init_DebugPhysics_skin_friction_coeff
{
public:
  explicit Init_DebugPhysics_skin_friction_coeff(::stonefish_ros2::msg::DebugPhysics & msg)
  : msg_(msg)
  {}
  Init_DebugPhysics_velocity skin_friction_coeff(::stonefish_ros2::msg::DebugPhysics::_skin_friction_coeff_type arg)
  {
    msg_.skin_friction_coeff = std::move(arg);
    return Init_DebugPhysics_velocity(msg_);
  }

private:
  ::stonefish_ros2::msg::DebugPhysics msg_;
};

class Init_DebugPhysics_damping_coeff
{
public:
  explicit Init_DebugPhysics_damping_coeff(::stonefish_ros2::msg::DebugPhysics & msg)
  : msg_(msg)
  {}
  Init_DebugPhysics_skin_friction_coeff damping_coeff(::stonefish_ros2::msg::DebugPhysics::_damping_coeff_type arg)
  {
    msg_.damping_coeff = std::move(arg);
    return Init_DebugPhysics_skin_friction_coeff(msg_);
  }

private:
  ::stonefish_ros2::msg::DebugPhysics msg_;
};

class Init_DebugPhysics_inertia
{
public:
  explicit Init_DebugPhysics_inertia(::stonefish_ros2::msg::DebugPhysics & msg)
  : msg_(msg)
  {}
  Init_DebugPhysics_damping_coeff inertia(::stonefish_ros2::msg::DebugPhysics::_inertia_type arg)
  {
    msg_.inertia = std::move(arg);
    return Init_DebugPhysics_damping_coeff(msg_);
  }

private:
  ::stonefish_ros2::msg::DebugPhysics msg_;
};

class Init_DebugPhysics_cob
{
public:
  explicit Init_DebugPhysics_cob(::stonefish_ros2::msg::DebugPhysics & msg)
  : msg_(msg)
  {}
  Init_DebugPhysics_inertia cob(::stonefish_ros2::msg::DebugPhysics::_cob_type arg)
  {
    msg_.cob = std::move(arg);
    return Init_DebugPhysics_inertia(msg_);
  }

private:
  ::stonefish_ros2::msg::DebugPhysics msg_;
};

class Init_DebugPhysics_cog
{
public:
  explicit Init_DebugPhysics_cog(::stonefish_ros2::msg::DebugPhysics & msg)
  : msg_(msg)
  {}
  Init_DebugPhysics_cob cog(::stonefish_ros2::msg::DebugPhysics::_cog_type arg)
  {
    msg_.cog = std::move(arg);
    return Init_DebugPhysics_cob(msg_);
  }

private:
  ::stonefish_ros2::msg::DebugPhysics msg_;
};

class Init_DebugPhysics_surface
{
public:
  explicit Init_DebugPhysics_surface(::stonefish_ros2::msg::DebugPhysics & msg)
  : msg_(msg)
  {}
  Init_DebugPhysics_cog surface(::stonefish_ros2::msg::DebugPhysics::_surface_type arg)
  {
    msg_.surface = std::move(arg);
    return Init_DebugPhysics_cog(msg_);
  }

private:
  ::stonefish_ros2::msg::DebugPhysics msg_;
};

class Init_DebugPhysics_volume
{
public:
  explicit Init_DebugPhysics_volume(::stonefish_ros2::msg::DebugPhysics & msg)
  : msg_(msg)
  {}
  Init_DebugPhysics_surface volume(::stonefish_ros2::msg::DebugPhysics::_volume_type arg)
  {
    msg_.volume = std::move(arg);
    return Init_DebugPhysics_surface(msg_);
  }

private:
  ::stonefish_ros2::msg::DebugPhysics msg_;
};

class Init_DebugPhysics_mass
{
public:
  explicit Init_DebugPhysics_mass(::stonefish_ros2::msg::DebugPhysics & msg)
  : msg_(msg)
  {}
  Init_DebugPhysics_volume mass(::stonefish_ros2::msg::DebugPhysics::_mass_type arg)
  {
    msg_.mass = std::move(arg);
    return Init_DebugPhysics_volume(msg_);
  }

private:
  ::stonefish_ros2::msg::DebugPhysics msg_;
};

class Init_DebugPhysics_header
{
public:
  Init_DebugPhysics_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DebugPhysics_mass header(::stonefish_ros2::msg::DebugPhysics::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DebugPhysics_mass(msg_);
  }

private:
  ::stonefish_ros2::msg::DebugPhysics msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::stonefish_ros2::msg::DebugPhysics>()
{
  return stonefish_ros2::msg::builder::Init_DebugPhysics_header();
}

}  // namespace stonefish_ros2

#endif  // STONEFISH_ROS2__MSG__DETAIL__DEBUG_PHYSICS__BUILDER_HPP_
