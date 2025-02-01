// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from cola2_msgs:msg/BodyForceReq.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__BODY_FORCE_REQ__STRUCT_HPP_
#define COLA2_MSGS__MSG__DETAIL__BODY_FORCE_REQ__STRUCT_HPP_

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
// Member 'goal'
#include "cola2_msgs/msg/detail/goal_descriptor__struct.hpp"
// Member 'wrench'
#include "geometry_msgs/msg/detail/wrench__struct.hpp"
// Member 'disable_axis'
#include "cola2_msgs/msg/detail/bool6_axis__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__cola2_msgs__msg__BodyForceReq __attribute__((deprecated))
#else
# define DEPRECATED__cola2_msgs__msg__BodyForceReq __declspec(deprecated)
#endif

namespace cola2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BodyForceReq_
{
  using Type = BodyForceReq_<ContainerAllocator>;

  explicit BodyForceReq_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    goal(_init),
    wrench(_init),
    disable_axis(_init)
  {
    (void)_init;
  }

  explicit BodyForceReq_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    goal(_alloc, _init),
    wrench(_alloc, _init),
    disable_axis(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _goal_type =
    cola2_msgs::msg::GoalDescriptor_<ContainerAllocator>;
  _goal_type goal;
  using _wrench_type =
    geometry_msgs::msg::Wrench_<ContainerAllocator>;
  _wrench_type wrench;
  using _disable_axis_type =
    cola2_msgs::msg::Bool6Axis_<ContainerAllocator>;
  _disable_axis_type disable_axis;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__goal(
    const cola2_msgs::msg::GoalDescriptor_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }
  Type & set__wrench(
    const geometry_msgs::msg::Wrench_<ContainerAllocator> & _arg)
  {
    this->wrench = _arg;
    return *this;
  }
  Type & set__disable_axis(
    const cola2_msgs::msg::Bool6Axis_<ContainerAllocator> & _arg)
  {
    this->disable_axis = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    cola2_msgs::msg::BodyForceReq_<ContainerAllocator> *;
  using ConstRawPtr =
    const cola2_msgs::msg::BodyForceReq_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cola2_msgs::msg::BodyForceReq_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cola2_msgs::msg::BodyForceReq_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::msg::BodyForceReq_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::msg::BodyForceReq_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::msg::BodyForceReq_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::msg::BodyForceReq_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cola2_msgs::msg::BodyForceReq_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cola2_msgs::msg::BodyForceReq_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cola2_msgs__msg__BodyForceReq
    std::shared_ptr<cola2_msgs::msg::BodyForceReq_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cola2_msgs__msg__BodyForceReq
    std::shared_ptr<cola2_msgs::msg::BodyForceReq_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BodyForceReq_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    if (this->wrench != other.wrench) {
      return false;
    }
    if (this->disable_axis != other.disable_axis) {
      return false;
    }
    return true;
  }
  bool operator!=(const BodyForceReq_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BodyForceReq_

// alias to use template instance with default allocator
using BodyForceReq =
  cola2_msgs::msg::BodyForceReq_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace cola2_msgs

#endif  // COLA2_MSGS__MSG__DETAIL__BODY_FORCE_REQ__STRUCT_HPP_
