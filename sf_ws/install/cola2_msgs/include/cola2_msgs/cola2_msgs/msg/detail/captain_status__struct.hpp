// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from cola2_msgs:msg/CaptainStatus.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__CAPTAIN_STATUS__STRUCT_HPP_
#define COLA2_MSGS__MSG__DETAIL__CAPTAIN_STATUS__STRUCT_HPP_

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
// Member 'loaded_missions'
#include "cola2_msgs/msg/detail/mission_state__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__cola2_msgs__msg__CaptainStatus __attribute__((deprecated))
#else
# define DEPRECATED__cola2_msgs__msg__CaptainStatus __declspec(deprecated)
#endif

namespace cola2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CaptainStatus_
{
  using Type = CaptainStatus_<ContainerAllocator>;

  explicit CaptainStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
      this->message = "";
    }
  }

  explicit CaptainStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
      this->message = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _state_type =
    uint8_t;
  _state_type state;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;
  using _loaded_missions_type =
    std::vector<cola2_msgs::msg::MissionState_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<cola2_msgs::msg::MissionState_<ContainerAllocator>>>;
  _loaded_missions_type loaded_missions;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__state(
    const uint8_t & _arg)
  {
    this->state = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }
  Type & set__loaded_missions(
    const std::vector<cola2_msgs::msg::MissionState_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<cola2_msgs::msg::MissionState_<ContainerAllocator>>> & _arg)
  {
    this->loaded_missions = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t IDLE =
    0u;
  static constexpr uint8_t GOTO =
    1u;
  static constexpr uint8_t MISSION =
    2u;
  static constexpr uint8_t KEEPPOSITION =
    3u;
  static constexpr uint8_t SAFETYKEEPPOSITION =
    4u;
  static constexpr uint8_t EXTERNALMISSION =
    5u;
  static constexpr uint8_t SECTION =
    6u;

  // pointer types
  using RawPtr =
    cola2_msgs::msg::CaptainStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const cola2_msgs::msg::CaptainStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cola2_msgs::msg::CaptainStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cola2_msgs::msg::CaptainStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::msg::CaptainStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::msg::CaptainStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::msg::CaptainStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::msg::CaptainStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cola2_msgs::msg::CaptainStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cola2_msgs::msg::CaptainStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cola2_msgs__msg__CaptainStatus
    std::shared_ptr<cola2_msgs::msg::CaptainStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cola2_msgs__msg__CaptainStatus
    std::shared_ptr<cola2_msgs::msg::CaptainStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CaptainStatus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    if (this->loaded_missions != other.loaded_missions) {
      return false;
    }
    return true;
  }
  bool operator!=(const CaptainStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CaptainStatus_

// alias to use template instance with default allocator
using CaptainStatus =
  cola2_msgs::msg::CaptainStatus_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CaptainStatus_<ContainerAllocator>::IDLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CaptainStatus_<ContainerAllocator>::GOTO;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CaptainStatus_<ContainerAllocator>::MISSION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CaptainStatus_<ContainerAllocator>::KEEPPOSITION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CaptainStatus_<ContainerAllocator>::SAFETYKEEPPOSITION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CaptainStatus_<ContainerAllocator>::EXTERNALMISSION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CaptainStatus_<ContainerAllocator>::SECTION;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace cola2_msgs

#endif  // COLA2_MSGS__MSG__DETAIL__CAPTAIN_STATUS__STRUCT_HPP_
