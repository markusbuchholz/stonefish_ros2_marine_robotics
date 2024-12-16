// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from stonefish_ros2:msg/DebugPhysics.idl
// generated code does not contain a copyright notice

#ifndef STONEFISH_ROS2__MSG__DETAIL__DEBUG_PHYSICS__STRUCT_HPP_
#define STONEFISH_ROS2__MSG__DETAIL__DEBUG_PHYSICS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'cog'
// Member 'cob'
// Member 'inertia'
// Member 'damping_coeff'
// Member 'skin_friction_coeff'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"
// Member 'velocity'
#include "geometry_msgs/msg/detail/twist__struct.hpp"
// Member 'buoyancy'
// Member 'damping'
// Member 'skin_friction'
#include "geometry_msgs/msg/detail/wrench__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__stonefish_ros2__msg__DebugPhysics __attribute__((deprecated))
#else
# define DEPRECATED__stonefish_ros2__msg__DebugPhysics __declspec(deprecated)
#endif

namespace stonefish_ros2
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DebugPhysics_
{
  using Type = DebugPhysics_<ContainerAllocator>;

  explicit DebugPhysics_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    cog(_init),
    cob(_init),
    inertia(_init),
    damping_coeff(_init),
    skin_friction_coeff(_init),
    velocity(_init),
    buoyancy(_init),
    damping(_init),
    skin_friction(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mass = 0.0;
      this->volume = 0.0;
      this->surface = 0.0;
      this->wetted_surface = 0.0;
      this->submerged_volume = 0.0;
    }
  }

  explicit DebugPhysics_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    cog(_alloc, _init),
    cob(_alloc, _init),
    inertia(_alloc, _init),
    damping_coeff(_alloc, _init),
    skin_friction_coeff(_alloc, _init),
    velocity(_alloc, _init),
    buoyancy(_alloc, _init),
    damping(_alloc, _init),
    skin_friction(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mass = 0.0;
      this->volume = 0.0;
      this->surface = 0.0;
      this->wetted_surface = 0.0;
      this->submerged_volume = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _mass_type =
    double;
  _mass_type mass;
  using _volume_type =
    double;
  _volume_type volume;
  using _surface_type =
    double;
  _surface_type surface;
  using _cog_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _cog_type cog;
  using _cob_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _cob_type cob;
  using _inertia_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _inertia_type inertia;
  using _damping_coeff_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _damping_coeff_type damping_coeff;
  using _skin_friction_coeff_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _skin_friction_coeff_type skin_friction_coeff;
  using _velocity_type =
    geometry_msgs::msg::Twist_<ContainerAllocator>;
  _velocity_type velocity;
  using _buoyancy_type =
    geometry_msgs::msg::Wrench_<ContainerAllocator>;
  _buoyancy_type buoyancy;
  using _damping_type =
    geometry_msgs::msg::Wrench_<ContainerAllocator>;
  _damping_type damping;
  using _skin_friction_type =
    geometry_msgs::msg::Wrench_<ContainerAllocator>;
  _skin_friction_type skin_friction;
  using _wetted_surface_type =
    double;
  _wetted_surface_type wetted_surface;
  using _submerged_volume_type =
    double;
  _submerged_volume_type submerged_volume;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__mass(
    const double & _arg)
  {
    this->mass = _arg;
    return *this;
  }
  Type & set__volume(
    const double & _arg)
  {
    this->volume = _arg;
    return *this;
  }
  Type & set__surface(
    const double & _arg)
  {
    this->surface = _arg;
    return *this;
  }
  Type & set__cog(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->cog = _arg;
    return *this;
  }
  Type & set__cob(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->cob = _arg;
    return *this;
  }
  Type & set__inertia(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->inertia = _arg;
    return *this;
  }
  Type & set__damping_coeff(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->damping_coeff = _arg;
    return *this;
  }
  Type & set__skin_friction_coeff(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->skin_friction_coeff = _arg;
    return *this;
  }
  Type & set__velocity(
    const geometry_msgs::msg::Twist_<ContainerAllocator> & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__buoyancy(
    const geometry_msgs::msg::Wrench_<ContainerAllocator> & _arg)
  {
    this->buoyancy = _arg;
    return *this;
  }
  Type & set__damping(
    const geometry_msgs::msg::Wrench_<ContainerAllocator> & _arg)
  {
    this->damping = _arg;
    return *this;
  }
  Type & set__skin_friction(
    const geometry_msgs::msg::Wrench_<ContainerAllocator> & _arg)
  {
    this->skin_friction = _arg;
    return *this;
  }
  Type & set__wetted_surface(
    const double & _arg)
  {
    this->wetted_surface = _arg;
    return *this;
  }
  Type & set__submerged_volume(
    const double & _arg)
  {
    this->submerged_volume = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    stonefish_ros2::msg::DebugPhysics_<ContainerAllocator> *;
  using ConstRawPtr =
    const stonefish_ros2::msg::DebugPhysics_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<stonefish_ros2::msg::DebugPhysics_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<stonefish_ros2::msg::DebugPhysics_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      stonefish_ros2::msg::DebugPhysics_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<stonefish_ros2::msg::DebugPhysics_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      stonefish_ros2::msg::DebugPhysics_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<stonefish_ros2::msg::DebugPhysics_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<stonefish_ros2::msg::DebugPhysics_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<stonefish_ros2::msg::DebugPhysics_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__stonefish_ros2__msg__DebugPhysics
    std::shared_ptr<stonefish_ros2::msg::DebugPhysics_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__stonefish_ros2__msg__DebugPhysics
    std::shared_ptr<stonefish_ros2::msg::DebugPhysics_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DebugPhysics_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->mass != other.mass) {
      return false;
    }
    if (this->volume != other.volume) {
      return false;
    }
    if (this->surface != other.surface) {
      return false;
    }
    if (this->cog != other.cog) {
      return false;
    }
    if (this->cob != other.cob) {
      return false;
    }
    if (this->inertia != other.inertia) {
      return false;
    }
    if (this->damping_coeff != other.damping_coeff) {
      return false;
    }
    if (this->skin_friction_coeff != other.skin_friction_coeff) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->buoyancy != other.buoyancy) {
      return false;
    }
    if (this->damping != other.damping) {
      return false;
    }
    if (this->skin_friction != other.skin_friction) {
      return false;
    }
    if (this->wetted_surface != other.wetted_surface) {
      return false;
    }
    if (this->submerged_volume != other.submerged_volume) {
      return false;
    }
    return true;
  }
  bool operator!=(const DebugPhysics_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DebugPhysics_

// alias to use template instance with default allocator
using DebugPhysics =
  stonefish_ros2::msg::DebugPhysics_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace stonefish_ros2

#endif  // STONEFISH_ROS2__MSG__DETAIL__DEBUG_PHYSICS__STRUCT_HPP_
