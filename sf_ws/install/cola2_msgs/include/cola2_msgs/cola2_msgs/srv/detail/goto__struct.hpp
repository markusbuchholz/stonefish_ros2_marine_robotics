// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from cola2_msgs:srv/Goto.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__SRV__DETAIL__GOTO__STRUCT_HPP_
#define COLA2_MSGS__SRV__DETAIL__GOTO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__cola2_msgs__srv__Goto_Request __attribute__((deprecated))
#else
# define DEPRECATED__cola2_msgs__srv__Goto_Request __declspec(deprecated)
#endif

namespace cola2_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Goto_Request_
{
  using Type = Goto_Request_<ContainerAllocator>;

  explicit Goto_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->final_x = 0.0;
      this->final_y = 0.0;
      this->final_depth = 0.0;
      this->final_altitude = 0.0;
      this->reference = 0;
      this->heave_mode = 0ull;
      this->surge_velocity = 0.0;
      this->tolerance_xy = 0.0;
      this->timeout = 0.0;
      this->no_altitude_goes_up = false;
    }
  }

  explicit Goto_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->final_x = 0.0;
      this->final_y = 0.0;
      this->final_depth = 0.0;
      this->final_altitude = 0.0;
      this->reference = 0;
      this->heave_mode = 0ull;
      this->surge_velocity = 0.0;
      this->tolerance_xy = 0.0;
      this->timeout = 0.0;
      this->no_altitude_goes_up = false;
    }
  }

  // field types and members
  using _final_x_type =
    double;
  _final_x_type final_x;
  using _final_y_type =
    double;
  _final_y_type final_y;
  using _final_depth_type =
    double;
  _final_depth_type final_depth;
  using _final_altitude_type =
    double;
  _final_altitude_type final_altitude;
  using _reference_type =
    uint8_t;
  _reference_type reference;
  using _heave_mode_type =
    uint64_t;
  _heave_mode_type heave_mode;
  using _surge_velocity_type =
    double;
  _surge_velocity_type surge_velocity;
  using _tolerance_xy_type =
    double;
  _tolerance_xy_type tolerance_xy;
  using _timeout_type =
    double;
  _timeout_type timeout;
  using _no_altitude_goes_up_type =
    bool;
  _no_altitude_goes_up_type no_altitude_goes_up;

  // setters for named parameter idiom
  Type & set__final_x(
    const double & _arg)
  {
    this->final_x = _arg;
    return *this;
  }
  Type & set__final_y(
    const double & _arg)
  {
    this->final_y = _arg;
    return *this;
  }
  Type & set__final_depth(
    const double & _arg)
  {
    this->final_depth = _arg;
    return *this;
  }
  Type & set__final_altitude(
    const double & _arg)
  {
    this->final_altitude = _arg;
    return *this;
  }
  Type & set__reference(
    const uint8_t & _arg)
  {
    this->reference = _arg;
    return *this;
  }
  Type & set__heave_mode(
    const uint64_t & _arg)
  {
    this->heave_mode = _arg;
    return *this;
  }
  Type & set__surge_velocity(
    const double & _arg)
  {
    this->surge_velocity = _arg;
    return *this;
  }
  Type & set__tolerance_xy(
    const double & _arg)
  {
    this->tolerance_xy = _arg;
    return *this;
  }
  Type & set__timeout(
    const double & _arg)
  {
    this->timeout = _arg;
    return *this;
  }
  Type & set__no_altitude_goes_up(
    const bool & _arg)
  {
    this->no_altitude_goes_up = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t NED =
    0u;
  static constexpr uint8_t GLOBAL =
    1u;
  static constexpr uint64_t DEPTH =
    0u;
  static constexpr uint64_t ALTITUDE =
    1u;
  static constexpr uint64_t BOTH =
    2u;

  // pointer types
  using RawPtr =
    cola2_msgs::srv::Goto_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const cola2_msgs::srv::Goto_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cola2_msgs::srv::Goto_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cola2_msgs::srv::Goto_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::srv::Goto_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::srv::Goto_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::srv::Goto_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::srv::Goto_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cola2_msgs::srv::Goto_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cola2_msgs::srv::Goto_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cola2_msgs__srv__Goto_Request
    std::shared_ptr<cola2_msgs::srv::Goto_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cola2_msgs__srv__Goto_Request
    std::shared_ptr<cola2_msgs::srv::Goto_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Goto_Request_ & other) const
  {
    if (this->final_x != other.final_x) {
      return false;
    }
    if (this->final_y != other.final_y) {
      return false;
    }
    if (this->final_depth != other.final_depth) {
      return false;
    }
    if (this->final_altitude != other.final_altitude) {
      return false;
    }
    if (this->reference != other.reference) {
      return false;
    }
    if (this->heave_mode != other.heave_mode) {
      return false;
    }
    if (this->surge_velocity != other.surge_velocity) {
      return false;
    }
    if (this->tolerance_xy != other.tolerance_xy) {
      return false;
    }
    if (this->timeout != other.timeout) {
      return false;
    }
    if (this->no_altitude_goes_up != other.no_altitude_goes_up) {
      return false;
    }
    return true;
  }
  bool operator!=(const Goto_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Goto_Request_

// alias to use template instance with default allocator
using Goto_Request =
  cola2_msgs::srv::Goto_Request_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Goto_Request_<ContainerAllocator>::NED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Goto_Request_<ContainerAllocator>::GLOBAL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint64_t Goto_Request_<ContainerAllocator>::DEPTH;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint64_t Goto_Request_<ContainerAllocator>::ALTITUDE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint64_t Goto_Request_<ContainerAllocator>::BOTH;
#endif  // __cplusplus < 201703L

}  // namespace srv

}  // namespace cola2_msgs


#ifndef _WIN32
# define DEPRECATED__cola2_msgs__srv__Goto_Response __attribute__((deprecated))
#else
# define DEPRECATED__cola2_msgs__srv__Goto_Response __declspec(deprecated)
#endif

namespace cola2_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Goto_Response_
{
  using Type = Goto_Response_<ContainerAllocator>;

  explicit Goto_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit Goto_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    cola2_msgs::srv::Goto_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const cola2_msgs::srv::Goto_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cola2_msgs::srv::Goto_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cola2_msgs::srv::Goto_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::srv::Goto_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::srv::Goto_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::srv::Goto_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::srv::Goto_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cola2_msgs::srv::Goto_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cola2_msgs::srv::Goto_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cola2_msgs__srv__Goto_Response
    std::shared_ptr<cola2_msgs::srv::Goto_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cola2_msgs__srv__Goto_Response
    std::shared_ptr<cola2_msgs::srv::Goto_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Goto_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const Goto_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Goto_Response_

// alias to use template instance with default allocator
using Goto_Response =
  cola2_msgs::srv::Goto_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace cola2_msgs

namespace cola2_msgs
{

namespace srv
{

struct Goto
{
  using Request = cola2_msgs::srv::Goto_Request;
  using Response = cola2_msgs::srv::Goto_Response;
};

}  // namespace srv

}  // namespace cola2_msgs

#endif  // COLA2_MSGS__SRV__DETAIL__GOTO__STRUCT_HPP_
