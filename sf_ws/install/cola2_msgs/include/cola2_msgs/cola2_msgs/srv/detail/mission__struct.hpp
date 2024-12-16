// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from cola2_msgs:srv/Mission.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__SRV__DETAIL__MISSION__STRUCT_HPP_
#define COLA2_MSGS__SRV__DETAIL__MISSION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__cola2_msgs__srv__Mission_Request __attribute__((deprecated))
#else
# define DEPRECATED__cola2_msgs__srv__Mission_Request __declspec(deprecated)
#endif

namespace cola2_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Mission_Request_
{
  using Type = Mission_Request_<ContainerAllocator>;

  explicit Mission_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mission = "";
    }
  }

  explicit Mission_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : mission(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mission = "";
    }
  }

  // field types and members
  using _mission_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _mission_type mission;

  // setters for named parameter idiom
  Type & set__mission(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->mission = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    cola2_msgs::srv::Mission_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const cola2_msgs::srv::Mission_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cola2_msgs::srv::Mission_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cola2_msgs::srv::Mission_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::srv::Mission_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::srv::Mission_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::srv::Mission_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::srv::Mission_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cola2_msgs::srv::Mission_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cola2_msgs::srv::Mission_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cola2_msgs__srv__Mission_Request
    std::shared_ptr<cola2_msgs::srv::Mission_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cola2_msgs__srv__Mission_Request
    std::shared_ptr<cola2_msgs::srv::Mission_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Mission_Request_ & other) const
  {
    if (this->mission != other.mission) {
      return false;
    }
    return true;
  }
  bool operator!=(const Mission_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Mission_Request_

// alias to use template instance with default allocator
using Mission_Request =
  cola2_msgs::srv::Mission_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace cola2_msgs


#ifndef _WIN32
# define DEPRECATED__cola2_msgs__srv__Mission_Response __attribute__((deprecated))
#else
# define DEPRECATED__cola2_msgs__srv__Mission_Response __declspec(deprecated)
#endif

namespace cola2_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Mission_Response_
{
  using Type = Mission_Response_<ContainerAllocator>;

  explicit Mission_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit Mission_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    cola2_msgs::srv::Mission_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const cola2_msgs::srv::Mission_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cola2_msgs::srv::Mission_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cola2_msgs::srv::Mission_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::srv::Mission_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::srv::Mission_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::srv::Mission_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::srv::Mission_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cola2_msgs::srv::Mission_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cola2_msgs::srv::Mission_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cola2_msgs__srv__Mission_Response
    std::shared_ptr<cola2_msgs::srv::Mission_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cola2_msgs__srv__Mission_Response
    std::shared_ptr<cola2_msgs::srv::Mission_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Mission_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const Mission_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Mission_Response_

// alias to use template instance with default allocator
using Mission_Response =
  cola2_msgs::srv::Mission_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace cola2_msgs

namespace cola2_msgs
{

namespace srv
{

struct Mission
{
  using Request = cola2_msgs::srv::Mission_Request;
  using Response = cola2_msgs::srv::Mission_Response;
};

}  // namespace srv

}  // namespace cola2_msgs

#endif  // COLA2_MSGS__SRV__DETAIL__MISSION__STRUCT_HPP_
