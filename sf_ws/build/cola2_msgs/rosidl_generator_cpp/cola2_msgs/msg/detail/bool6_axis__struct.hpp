// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from cola2_msgs:msg/Bool6Axis.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__BOOL6_AXIS__STRUCT_HPP_
#define COLA2_MSGS__MSG__DETAIL__BOOL6_AXIS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__cola2_msgs__msg__Bool6Axis __attribute__((deprecated))
#else
# define DEPRECATED__cola2_msgs__msg__Bool6Axis __declspec(deprecated)
#endif

namespace cola2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Bool6Axis_
{
  using Type = Bool6Axis_<ContainerAllocator>;

  explicit Bool6Axis_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = false;
      this->y = false;
      this->z = false;
      this->roll = false;
      this->pitch = false;
      this->yaw = false;
    }
  }

  explicit Bool6Axis_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = false;
      this->y = false;
      this->z = false;
      this->roll = false;
      this->pitch = false;
      this->yaw = false;
    }
  }

  // field types and members
  using _x_type =
    bool;
  _x_type x;
  using _y_type =
    bool;
  _y_type y;
  using _z_type =
    bool;
  _z_type z;
  using _roll_type =
    bool;
  _roll_type roll;
  using _pitch_type =
    bool;
  _pitch_type pitch;
  using _yaw_type =
    bool;
  _yaw_type yaw;

  // setters for named parameter idiom
  Type & set__x(
    const bool & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const bool & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const bool & _arg)
  {
    this->z = _arg;
    return *this;
  }
  Type & set__roll(
    const bool & _arg)
  {
    this->roll = _arg;
    return *this;
  }
  Type & set__pitch(
    const bool & _arg)
  {
    this->pitch = _arg;
    return *this;
  }
  Type & set__yaw(
    const bool & _arg)
  {
    this->yaw = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    cola2_msgs::msg::Bool6Axis_<ContainerAllocator> *;
  using ConstRawPtr =
    const cola2_msgs::msg::Bool6Axis_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cola2_msgs::msg::Bool6Axis_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cola2_msgs::msg::Bool6Axis_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::msg::Bool6Axis_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::msg::Bool6Axis_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::msg::Bool6Axis_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::msg::Bool6Axis_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cola2_msgs::msg::Bool6Axis_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cola2_msgs::msg::Bool6Axis_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cola2_msgs__msg__Bool6Axis
    std::shared_ptr<cola2_msgs::msg::Bool6Axis_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cola2_msgs__msg__Bool6Axis
    std::shared_ptr<cola2_msgs::msg::Bool6Axis_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Bool6Axis_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->z != other.z) {
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
  bool operator!=(const Bool6Axis_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Bool6Axis_

// alias to use template instance with default allocator
using Bool6Axis =
  cola2_msgs::msg::Bool6Axis_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace cola2_msgs

#endif  // COLA2_MSGS__MSG__DETAIL__BOOL6_AXIS__STRUCT_HPP_
