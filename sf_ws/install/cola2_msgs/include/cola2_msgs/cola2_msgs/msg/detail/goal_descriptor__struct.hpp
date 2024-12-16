// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from cola2_msgs:msg/GoalDescriptor.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__GOAL_DESCRIPTOR__STRUCT_HPP_
#define COLA2_MSGS__MSG__DETAIL__GOAL_DESCRIPTOR__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__cola2_msgs__msg__GoalDescriptor __attribute__((deprecated))
#else
# define DEPRECATED__cola2_msgs__msg__GoalDescriptor __declspec(deprecated)
#endif

namespace cola2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GoalDescriptor_
{
  using Type = GoalDescriptor_<ContainerAllocator>;

  explicit GoalDescriptor_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->requester = "";
      this->priority = 0ul;
    }
  }

  explicit GoalDescriptor_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : requester(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->requester = "";
      this->priority = 0ul;
    }
  }

  // field types and members
  using _requester_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _requester_type requester;
  using _priority_type =
    uint32_t;
  _priority_type priority;

  // setters for named parameter idiom
  Type & set__requester(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->requester = _arg;
    return *this;
  }
  Type & set__priority(
    const uint32_t & _arg)
  {
    this->priority = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint32_t PRIORITY_TELEOPERATION_LOW =
    0u;
  static constexpr uint32_t PRIORITY_SAFETY_LOW =
    5u;
  static constexpr uint32_t PRIORITY_NORMAL =
    10u;
  static constexpr uint32_t PRIORITY_SAFETY =
    30u;
  static constexpr uint32_t PRIORITY_TELEOPERATION =
    40u;
  static constexpr uint32_t PRIORITY_SAFETY_HIGH =
    50u;
  static constexpr uint32_t PRIORITY_TELEOPERATION_HIGH =
    60u;

  // pointer types
  using RawPtr =
    cola2_msgs::msg::GoalDescriptor_<ContainerAllocator> *;
  using ConstRawPtr =
    const cola2_msgs::msg::GoalDescriptor_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cola2_msgs::msg::GoalDescriptor_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cola2_msgs::msg::GoalDescriptor_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::msg::GoalDescriptor_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::msg::GoalDescriptor_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::msg::GoalDescriptor_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::msg::GoalDescriptor_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cola2_msgs::msg::GoalDescriptor_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cola2_msgs::msg::GoalDescriptor_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cola2_msgs__msg__GoalDescriptor
    std::shared_ptr<cola2_msgs::msg::GoalDescriptor_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cola2_msgs__msg__GoalDescriptor
    std::shared_ptr<cola2_msgs::msg::GoalDescriptor_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GoalDescriptor_ & other) const
  {
    if (this->requester != other.requester) {
      return false;
    }
    if (this->priority != other.priority) {
      return false;
    }
    return true;
  }
  bool operator!=(const GoalDescriptor_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GoalDescriptor_

// alias to use template instance with default allocator
using GoalDescriptor =
  cola2_msgs::msg::GoalDescriptor_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t GoalDescriptor_<ContainerAllocator>::PRIORITY_TELEOPERATION_LOW;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t GoalDescriptor_<ContainerAllocator>::PRIORITY_SAFETY_LOW;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t GoalDescriptor_<ContainerAllocator>::PRIORITY_NORMAL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t GoalDescriptor_<ContainerAllocator>::PRIORITY_SAFETY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t GoalDescriptor_<ContainerAllocator>::PRIORITY_TELEOPERATION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t GoalDescriptor_<ContainerAllocator>::PRIORITY_SAFETY_HIGH;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t GoalDescriptor_<ContainerAllocator>::PRIORITY_TELEOPERATION_HIGH;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace cola2_msgs

#endif  // COLA2_MSGS__MSG__DETAIL__GOAL_DESCRIPTOR__STRUCT_HPP_
