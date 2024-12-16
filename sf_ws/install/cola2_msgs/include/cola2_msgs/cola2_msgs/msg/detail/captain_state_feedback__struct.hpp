// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from cola2_msgs:msg/CaptainStateFeedback.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__CAPTAIN_STATE_FEEDBACK__STRUCT_HPP_
#define COLA2_MSGS__MSG__DETAIL__CAPTAIN_STATE_FEEDBACK__STRUCT_HPP_

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
// Member 'keyvalues'
#include "cola2_msgs/msg/detail/key_value__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__cola2_msgs__msg__CaptainStateFeedback __attribute__((deprecated))
#else
# define DEPRECATED__cola2_msgs__msg__CaptainStateFeedback __declspec(deprecated)
#endif

namespace cola2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CaptainStateFeedback_
{
  using Type = CaptainStateFeedback_<ContainerAllocator>;

  explicit CaptainStateFeedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
      this->time_to_finish = 0.0;
      this->name = "";
      this->id = 0;
    }
  }

  explicit CaptainStateFeedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
      this->time_to_finish = 0.0;
      this->name = "";
      this->id = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _state_type =
    uint8_t;
  _state_type state;
  using _time_to_finish_type =
    double;
  _time_to_finish_type time_to_finish;
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;
  using _id_type =
    uint16_t;
  _id_type id;
  using _keyvalues_type =
    std::vector<cola2_msgs::msg::KeyValue_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<cola2_msgs::msg::KeyValue_<ContainerAllocator>>>;
  _keyvalues_type keyvalues;

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
  Type & set__time_to_finish(
    const double & _arg)
  {
    this->time_to_finish = _arg;
    return *this;
  }
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__id(
    const uint16_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__keyvalues(
    const std::vector<cola2_msgs::msg::KeyValue_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<cola2_msgs::msg::KeyValue_<ContainerAllocator>>> & _arg)
  {
    this->keyvalues = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t ACTIVE =
    0u;
  static constexpr uint8_t SUCCESS =
    1u;
  static constexpr uint8_t FAILURE =
    2u;
  static constexpr uint8_t STOPPED =
    3u;

  // pointer types
  using RawPtr =
    cola2_msgs::msg::CaptainStateFeedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const cola2_msgs::msg::CaptainStateFeedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cola2_msgs::msg::CaptainStateFeedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cola2_msgs::msg::CaptainStateFeedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::msg::CaptainStateFeedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::msg::CaptainStateFeedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::msg::CaptainStateFeedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::msg::CaptainStateFeedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cola2_msgs::msg::CaptainStateFeedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cola2_msgs::msg::CaptainStateFeedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cola2_msgs__msg__CaptainStateFeedback
    std::shared_ptr<cola2_msgs::msg::CaptainStateFeedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cola2_msgs__msg__CaptainStateFeedback
    std::shared_ptr<cola2_msgs::msg::CaptainStateFeedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CaptainStateFeedback_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    if (this->time_to_finish != other.time_to_finish) {
      return false;
    }
    if (this->name != other.name) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    if (this->keyvalues != other.keyvalues) {
      return false;
    }
    return true;
  }
  bool operator!=(const CaptainStateFeedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CaptainStateFeedback_

// alias to use template instance with default allocator
using CaptainStateFeedback =
  cola2_msgs::msg::CaptainStateFeedback_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CaptainStateFeedback_<ContainerAllocator>::ACTIVE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CaptainStateFeedback_<ContainerAllocator>::SUCCESS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CaptainStateFeedback_<ContainerAllocator>::FAILURE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CaptainStateFeedback_<ContainerAllocator>::STOPPED;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace cola2_msgs

#endif  // COLA2_MSGS__MSG__DETAIL__CAPTAIN_STATE_FEEDBACK__STRUCT_HPP_
