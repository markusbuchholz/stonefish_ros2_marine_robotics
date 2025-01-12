// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from cola2_msgs:msg/RecoveryAction.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__RECOVERY_ACTION__STRUCT_HPP_
#define COLA2_MSGS__MSG__DETAIL__RECOVERY_ACTION__STRUCT_HPP_

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
# define DEPRECATED__cola2_msgs__msg__RecoveryAction __attribute__((deprecated))
#else
# define DEPRECATED__cola2_msgs__msg__RecoveryAction __declspec(deprecated)
#endif

namespace cola2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RecoveryAction_
{
  using Type = RecoveryAction_<ContainerAllocator>;

  explicit RecoveryAction_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->error_level = 0;
      this->error_string = "";
    }
  }

  explicit RecoveryAction_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    error_string(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->error_level = 0;
      this->error_string = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _error_level_type =
    uint16_t;
  _error_level_type error_level;
  using _error_string_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _error_string_type error_string;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__error_level(
    const uint16_t & _arg)
  {
    this->error_level = _arg;
    return *this;
  }
  Type & set__error_string(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->error_string = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint16_t NONE =
    0u;
  static constexpr uint16_t INFORMATIVE =
    1u;
  static constexpr uint16_t ABORT =
    2u;
  static constexpr uint16_t ABORT_AND_SURFACE =
    3u;
  static constexpr uint16_t EMERGENCY_SURFACE =
    4u;
  static constexpr uint16_t DROP_WEIGHT =
    5u;

  // pointer types
  using RawPtr =
    cola2_msgs::msg::RecoveryAction_<ContainerAllocator> *;
  using ConstRawPtr =
    const cola2_msgs::msg::RecoveryAction_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cola2_msgs::msg::RecoveryAction_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cola2_msgs::msg::RecoveryAction_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::msg::RecoveryAction_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::msg::RecoveryAction_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::msg::RecoveryAction_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::msg::RecoveryAction_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cola2_msgs::msg::RecoveryAction_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cola2_msgs::msg::RecoveryAction_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cola2_msgs__msg__RecoveryAction
    std::shared_ptr<cola2_msgs::msg::RecoveryAction_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cola2_msgs__msg__RecoveryAction
    std::shared_ptr<cola2_msgs::msg::RecoveryAction_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RecoveryAction_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->error_level != other.error_level) {
      return false;
    }
    if (this->error_string != other.error_string) {
      return false;
    }
    return true;
  }
  bool operator!=(const RecoveryAction_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RecoveryAction_

// alias to use template instance with default allocator
using RecoveryAction =
  cola2_msgs::msg::RecoveryAction_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t RecoveryAction_<ContainerAllocator>::NONE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t RecoveryAction_<ContainerAllocator>::INFORMATIVE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t RecoveryAction_<ContainerAllocator>::ABORT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t RecoveryAction_<ContainerAllocator>::ABORT_AND_SURFACE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t RecoveryAction_<ContainerAllocator>::EMERGENCY_SURFACE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t RecoveryAction_<ContainerAllocator>::DROP_WEIGHT;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace cola2_msgs

#endif  // COLA2_MSGS__MSG__DETAIL__RECOVERY_ACTION__STRUCT_HPP_
