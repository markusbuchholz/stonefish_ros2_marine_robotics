// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from cola2_msgs:srv/Recovery.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__SRV__DETAIL__RECOVERY__STRUCT_HPP_
#define COLA2_MSGS__SRV__DETAIL__RECOVERY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'requested_action'
#include "cola2_msgs/msg/detail/recovery_action__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__cola2_msgs__srv__Recovery_Request __attribute__((deprecated))
#else
# define DEPRECATED__cola2_msgs__srv__Recovery_Request __declspec(deprecated)
#endif

namespace cola2_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Recovery_Request_
{
  using Type = Recovery_Request_<ContainerAllocator>;

  explicit Recovery_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : requested_action(_init)
  {
    (void)_init;
  }

  explicit Recovery_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : requested_action(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _requested_action_type =
    cola2_msgs::msg::RecoveryAction_<ContainerAllocator>;
  _requested_action_type requested_action;

  // setters for named parameter idiom
  Type & set__requested_action(
    const cola2_msgs::msg::RecoveryAction_<ContainerAllocator> & _arg)
  {
    this->requested_action = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    cola2_msgs::srv::Recovery_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const cola2_msgs::srv::Recovery_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cola2_msgs::srv::Recovery_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cola2_msgs::srv::Recovery_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::srv::Recovery_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::srv::Recovery_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::srv::Recovery_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::srv::Recovery_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cola2_msgs::srv::Recovery_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cola2_msgs::srv::Recovery_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cola2_msgs__srv__Recovery_Request
    std::shared_ptr<cola2_msgs::srv::Recovery_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cola2_msgs__srv__Recovery_Request
    std::shared_ptr<cola2_msgs::srv::Recovery_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Recovery_Request_ & other) const
  {
    if (this->requested_action != other.requested_action) {
      return false;
    }
    return true;
  }
  bool operator!=(const Recovery_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Recovery_Request_

// alias to use template instance with default allocator
using Recovery_Request =
  cola2_msgs::srv::Recovery_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace cola2_msgs


#ifndef _WIN32
# define DEPRECATED__cola2_msgs__srv__Recovery_Response __attribute__((deprecated))
#else
# define DEPRECATED__cola2_msgs__srv__Recovery_Response __declspec(deprecated)
#endif

namespace cola2_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Recovery_Response_
{
  using Type = Recovery_Response_<ContainerAllocator>;

  explicit Recovery_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->attempted = false;
    }
  }

  explicit Recovery_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    cola2_msgs::srv::Recovery_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const cola2_msgs::srv::Recovery_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cola2_msgs::srv::Recovery_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cola2_msgs::srv::Recovery_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::srv::Recovery_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::srv::Recovery_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::srv::Recovery_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::srv::Recovery_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cola2_msgs::srv::Recovery_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cola2_msgs::srv::Recovery_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cola2_msgs__srv__Recovery_Response
    std::shared_ptr<cola2_msgs::srv::Recovery_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cola2_msgs__srv__Recovery_Response
    std::shared_ptr<cola2_msgs::srv::Recovery_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Recovery_Response_ & other) const
  {
    if (this->attempted != other.attempted) {
      return false;
    }
    return true;
  }
  bool operator!=(const Recovery_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Recovery_Response_

// alias to use template instance with default allocator
using Recovery_Response =
  cola2_msgs::srv::Recovery_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace cola2_msgs

namespace cola2_msgs
{

namespace srv
{

struct Recovery
{
  using Request = cola2_msgs::srv::Recovery_Request;
  using Response = cola2_msgs::srv::Recovery_Response;
};

}  // namespace srv

}  // namespace cola2_msgs

#endif  // COLA2_MSGS__SRV__DETAIL__RECOVERY__STRUCT_HPP_
