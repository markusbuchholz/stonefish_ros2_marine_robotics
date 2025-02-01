// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from cola2_msgs:msg/SafetySupervisorStatus.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__SAFETY_SUPERVISOR_STATUS__STRUCT_HPP_
#define COLA2_MSGS__MSG__DETAIL__SAFETY_SUPERVISOR_STATUS__STRUCT_HPP_

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
// Member 'recovery_action'
#include "cola2_msgs/msg/detail/recovery_action__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__cola2_msgs__msg__SafetySupervisorStatus __attribute__((deprecated))
#else
# define DEPRECATED__cola2_msgs__msg__SafetySupervisorStatus __declspec(deprecated)
#endif

namespace cola2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SafetySupervisorStatus_
{
  using Type = SafetySupervisorStatus_<ContainerAllocator>;

  explicit SafetySupervisorStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    recovery_action(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status_code = 0ul;
    }
  }

  explicit SafetySupervisorStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    recovery_action(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status_code = 0ul;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _status_code_type =
    uint32_t;
  _status_code_type status_code;
  using _recovery_action_type =
    cola2_msgs::msg::RecoveryAction_<ContainerAllocator>;
  _recovery_action_type recovery_action;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__status_code(
    const uint32_t & _arg)
  {
    this->status_code = _arg;
    return *this;
  }
  Type & set__recovery_action(
    const cola2_msgs::msg::RecoveryAction_<ContainerAllocator> & _arg)
  {
    this->recovery_action = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    cola2_msgs::msg::SafetySupervisorStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const cola2_msgs::msg::SafetySupervisorStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cola2_msgs::msg::SafetySupervisorStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cola2_msgs::msg::SafetySupervisorStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::msg::SafetySupervisorStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::msg::SafetySupervisorStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::msg::SafetySupervisorStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::msg::SafetySupervisorStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cola2_msgs::msg::SafetySupervisorStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cola2_msgs::msg::SafetySupervisorStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cola2_msgs__msg__SafetySupervisorStatus
    std::shared_ptr<cola2_msgs::msg::SafetySupervisorStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cola2_msgs__msg__SafetySupervisorStatus
    std::shared_ptr<cola2_msgs::msg::SafetySupervisorStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SafetySupervisorStatus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->status_code != other.status_code) {
      return false;
    }
    if (this->recovery_action != other.recovery_action) {
      return false;
    }
    return true;
  }
  bool operator!=(const SafetySupervisorStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SafetySupervisorStatus_

// alias to use template instance with default allocator
using SafetySupervisorStatus =
  cola2_msgs::msg::SafetySupervisorStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace cola2_msgs

#endif  // COLA2_MSGS__MSG__DETAIL__SAFETY_SUPERVISOR_STATUS__STRUCT_HPP_
