// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from cola2_msgs:msg/MissionState.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__MISSION_STATE__STRUCT_HPP_
#define COLA2_MSGS__MSG__DETAIL__MISSION_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'last_active'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__cola2_msgs__msg__MissionState __attribute__((deprecated))
#else
# define DEPRECATED__cola2_msgs__msg__MissionState __declspec(deprecated)
#endif

namespace cola2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MissionState_
{
  using Type = MissionState_<ContainerAllocator>;

  explicit MissionState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : last_active(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->current_step = 0l;
    }
  }

  explicit MissionState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : name(_alloc),
    last_active(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->current_step = 0l;
    }
  }

  // field types and members
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;
  using _current_step_type =
    int32_t;
  _current_step_type current_step;
  using _last_active_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _last_active_type last_active;

  // setters for named parameter idiom
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__current_step(
    const int32_t & _arg)
  {
    this->current_step = _arg;
    return *this;
  }
  Type & set__last_active(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->last_active = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    cola2_msgs::msg::MissionState_<ContainerAllocator> *;
  using ConstRawPtr =
    const cola2_msgs::msg::MissionState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cola2_msgs::msg::MissionState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cola2_msgs::msg::MissionState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::msg::MissionState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::msg::MissionState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::msg::MissionState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::msg::MissionState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cola2_msgs::msg::MissionState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cola2_msgs::msg::MissionState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cola2_msgs__msg__MissionState
    std::shared_ptr<cola2_msgs::msg::MissionState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cola2_msgs__msg__MissionState
    std::shared_ptr<cola2_msgs::msg::MissionState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MissionState_ & other) const
  {
    if (this->name != other.name) {
      return false;
    }
    if (this->current_step != other.current_step) {
      return false;
    }
    if (this->last_active != other.last_active) {
      return false;
    }
    return true;
  }
  bool operator!=(const MissionState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MissionState_

// alias to use template instance with default allocator
using MissionState =
  cola2_msgs::msg::MissionState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace cola2_msgs

#endif  // COLA2_MSGS__MSG__DETAIL__MISSION_STATE__STRUCT_HPP_
