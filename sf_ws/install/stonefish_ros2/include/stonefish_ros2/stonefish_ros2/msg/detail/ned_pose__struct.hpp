// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from stonefish_ros2:msg/NEDPose.idl
// generated code does not contain a copyright notice

#ifndef STONEFISH_ROS2__MSG__DETAIL__NED_POSE__STRUCT_HPP_
#define STONEFISH_ROS2__MSG__DETAIL__NED_POSE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__stonefish_ros2__msg__NEDPose __attribute__((deprecated))
#else
# define DEPRECATED__stonefish_ros2__msg__NEDPose __declspec(deprecated)
#endif

namespace stonefish_ros2
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct NEDPose_
{
  using Type = NEDPose_<ContainerAllocator>;

  explicit NEDPose_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->north = 0.0;
      this->east = 0.0;
      this->down = 0.0;
      this->roll = 0.0;
      this->pitch = 0.0;
      this->yaw = 0.0;
    }
  }

  explicit NEDPose_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->north = 0.0;
      this->east = 0.0;
      this->down = 0.0;
      this->roll = 0.0;
      this->pitch = 0.0;
      this->yaw = 0.0;
    }
  }

  // field types and members
  using _north_type =
    double;
  _north_type north;
  using _east_type =
    double;
  _east_type east;
  using _down_type =
    double;
  _down_type down;
  using _roll_type =
    double;
  _roll_type roll;
  using _pitch_type =
    double;
  _pitch_type pitch;
  using _yaw_type =
    double;
  _yaw_type yaw;

  // setters for named parameter idiom
  Type & set__north(
    const double & _arg)
  {
    this->north = _arg;
    return *this;
  }
  Type & set__east(
    const double & _arg)
  {
    this->east = _arg;
    return *this;
  }
  Type & set__down(
    const double & _arg)
  {
    this->down = _arg;
    return *this;
  }
  Type & set__roll(
    const double & _arg)
  {
    this->roll = _arg;
    return *this;
  }
  Type & set__pitch(
    const double & _arg)
  {
    this->pitch = _arg;
    return *this;
  }
  Type & set__yaw(
    const double & _arg)
  {
    this->yaw = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    stonefish_ros2::msg::NEDPose_<ContainerAllocator> *;
  using ConstRawPtr =
    const stonefish_ros2::msg::NEDPose_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<stonefish_ros2::msg::NEDPose_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<stonefish_ros2::msg::NEDPose_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      stonefish_ros2::msg::NEDPose_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<stonefish_ros2::msg::NEDPose_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      stonefish_ros2::msg::NEDPose_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<stonefish_ros2::msg::NEDPose_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<stonefish_ros2::msg::NEDPose_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<stonefish_ros2::msg::NEDPose_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__stonefish_ros2__msg__NEDPose
    std::shared_ptr<stonefish_ros2::msg::NEDPose_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__stonefish_ros2__msg__NEDPose
    std::shared_ptr<stonefish_ros2::msg::NEDPose_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NEDPose_ & other) const
  {
    if (this->north != other.north) {
      return false;
    }
    if (this->east != other.east) {
      return false;
    }
    if (this->down != other.down) {
      return false;
    }
    if (this->roll != other.roll) {
      return false;
    }
    if (this->pitch != other.pitch) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    return true;
  }
  bool operator!=(const NEDPose_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NEDPose_

// alias to use template instance with default allocator
using NEDPose =
  stonefish_ros2::msg::NEDPose_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace stonefish_ros2

#endif  // STONEFISH_ROS2__MSG__DETAIL__NED_POSE__STRUCT_HPP_
