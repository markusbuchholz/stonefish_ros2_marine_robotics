// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from cola2_msgs:srv/MaxJoyVelocity.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__SRV__DETAIL__MAX_JOY_VELOCITY__STRUCT_HPP_
#define COLA2_MSGS__SRV__DETAIL__MAX_JOY_VELOCITY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__cola2_msgs__srv__MaxJoyVelocity_Request __attribute__((deprecated))
#else
# define DEPRECATED__cola2_msgs__srv__MaxJoyVelocity_Request __declspec(deprecated)
#endif

namespace cola2_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MaxJoyVelocity_Request_
{
  using Type = MaxJoyVelocity_Request_<ContainerAllocator>;

  explicit MaxJoyVelocity_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 6>::iterator, double>(this->max_joy_velocity.begin(), this->max_joy_velocity.end(), 0.0);
    }
  }

  explicit MaxJoyVelocity_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : max_joy_velocity(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 6>::iterator, double>(this->max_joy_velocity.begin(), this->max_joy_velocity.end(), 0.0);
    }
  }

  // field types and members
  using _max_joy_velocity_type =
    std::array<double, 6>;
  _max_joy_velocity_type max_joy_velocity;

  // setters for named parameter idiom
  Type & set__max_joy_velocity(
    const std::array<double, 6> & _arg)
  {
    this->max_joy_velocity = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    cola2_msgs::srv::MaxJoyVelocity_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const cola2_msgs::srv::MaxJoyVelocity_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cola2_msgs::srv::MaxJoyVelocity_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cola2_msgs::srv::MaxJoyVelocity_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::srv::MaxJoyVelocity_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::srv::MaxJoyVelocity_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::srv::MaxJoyVelocity_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::srv::MaxJoyVelocity_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cola2_msgs::srv::MaxJoyVelocity_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cola2_msgs::srv::MaxJoyVelocity_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cola2_msgs__srv__MaxJoyVelocity_Request
    std::shared_ptr<cola2_msgs::srv::MaxJoyVelocity_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cola2_msgs__srv__MaxJoyVelocity_Request
    std::shared_ptr<cola2_msgs::srv::MaxJoyVelocity_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MaxJoyVelocity_Request_ & other) const
  {
    if (this->max_joy_velocity != other.max_joy_velocity) {
      return false;
    }
    return true;
  }
  bool operator!=(const MaxJoyVelocity_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MaxJoyVelocity_Request_

// alias to use template instance with default allocator
using MaxJoyVelocity_Request =
  cola2_msgs::srv::MaxJoyVelocity_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace cola2_msgs


#ifndef _WIN32
# define DEPRECATED__cola2_msgs__srv__MaxJoyVelocity_Response __attribute__((deprecated))
#else
# define DEPRECATED__cola2_msgs__srv__MaxJoyVelocity_Response __declspec(deprecated)
#endif

namespace cola2_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MaxJoyVelocity_Response_
{
  using Type = MaxJoyVelocity_Response_<ContainerAllocator>;

  explicit MaxJoyVelocity_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->attempted = false;
    }
  }

  explicit MaxJoyVelocity_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->attempted = false;
    }
  }

  // field types and members
  using _attempted_type =
    bool;
  _attempted_type attempted;

  // setters for named parameter idiom
  Type & set__attempted(
    const bool & _arg)
  {
    this->attempted = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    cola2_msgs::srv::MaxJoyVelocity_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const cola2_msgs::srv::MaxJoyVelocity_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cola2_msgs::srv::MaxJoyVelocity_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cola2_msgs::srv::MaxJoyVelocity_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::srv::MaxJoyVelocity_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::srv::MaxJoyVelocity_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::srv::MaxJoyVelocity_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::srv::MaxJoyVelocity_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cola2_msgs::srv::MaxJoyVelocity_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cola2_msgs::srv::MaxJoyVelocity_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cola2_msgs__srv__MaxJoyVelocity_Response
    std::shared_ptr<cola2_msgs::srv::MaxJoyVelocity_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cola2_msgs__srv__MaxJoyVelocity_Response
    std::shared_ptr<cola2_msgs::srv::MaxJoyVelocity_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MaxJoyVelocity_Response_ & other) const
  {
    if (this->attempted != other.attempted) {
      return false;
    }
    return true;
  }
  bool operator!=(const MaxJoyVelocity_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MaxJoyVelocity_Response_

// alias to use template instance with default allocator
using MaxJoyVelocity_Response =
  cola2_msgs::srv::MaxJoyVelocity_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace cola2_msgs

namespace cola2_msgs
{

namespace srv
{

struct MaxJoyVelocity
{
  using Request = cola2_msgs::srv::MaxJoyVelocity_Request;
  using Response = cola2_msgs::srv::MaxJoyVelocity_Response;
};

}  // namespace srv

}  // namespace cola2_msgs

#endif  // COLA2_MSGS__SRV__DETAIL__MAX_JOY_VELOCITY__STRUCT_HPP_
