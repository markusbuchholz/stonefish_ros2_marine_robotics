// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from stonefish_ros2:msg/ThrusterState.idl
// generated code does not contain a copyright notice

#ifndef STONEFISH_ROS2__MSG__DETAIL__THRUSTER_STATE__STRUCT_HPP_
#define STONEFISH_ROS2__MSG__DETAIL__THRUSTER_STATE__STRUCT_HPP_

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

#ifndef _WIN32
# define DEPRECATED__stonefish_ros2__msg__ThrusterState __attribute__((deprecated))
#else
# define DEPRECATED__stonefish_ros2__msg__ThrusterState __declspec(deprecated)
#endif

namespace stonefish_ros2
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ThrusterState_
{
  using Type = ThrusterState_<ContainerAllocator>;

  explicit ThrusterState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit ThrusterState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _setpoint_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _setpoint_type setpoint;
  using _rpm_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _rpm_type rpm;
  using _thrust_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _thrust_type thrust;
  using _torque_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _torque_type torque;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__setpoint(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->setpoint = _arg;
    return *this;
  }
  Type & set__rpm(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->rpm = _arg;
    return *this;
  }
  Type & set__thrust(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->thrust = _arg;
    return *this;
  }
  Type & set__torque(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->torque = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    stonefish_ros2::msg::ThrusterState_<ContainerAllocator> *;
  using ConstRawPtr =
    const stonefish_ros2::msg::ThrusterState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<stonefish_ros2::msg::ThrusterState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<stonefish_ros2::msg::ThrusterState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      stonefish_ros2::msg::ThrusterState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<stonefish_ros2::msg::ThrusterState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      stonefish_ros2::msg::ThrusterState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<stonefish_ros2::msg::ThrusterState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<stonefish_ros2::msg::ThrusterState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<stonefish_ros2::msg::ThrusterState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__stonefish_ros2__msg__ThrusterState
    std::shared_ptr<stonefish_ros2::msg::ThrusterState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__stonefish_ros2__msg__ThrusterState
    std::shared_ptr<stonefish_ros2::msg::ThrusterState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ThrusterState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->setpoint != other.setpoint) {
      return false;
    }
    if (this->rpm != other.rpm) {
      return false;
    }
    if (this->thrust != other.thrust) {
      return false;
    }
    if (this->torque != other.torque) {
      return false;
    }
    return true;
  }
  bool operator!=(const ThrusterState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ThrusterState_

// alias to use template instance with default allocator
using ThrusterState =
  stonefish_ros2::msg::ThrusterState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace stonefish_ros2

#endif  // STONEFISH_ROS2__MSG__DETAIL__THRUSTER_STATE__STRUCT_HPP_
