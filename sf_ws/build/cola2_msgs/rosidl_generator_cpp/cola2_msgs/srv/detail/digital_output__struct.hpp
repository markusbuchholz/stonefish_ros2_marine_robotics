// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from cola2_msgs:srv/DigitalOutput.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__SRV__DETAIL__DIGITAL_OUTPUT__STRUCT_HPP_
#define COLA2_MSGS__SRV__DETAIL__DIGITAL_OUTPUT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__cola2_msgs__srv__DigitalOutput_Request __attribute__((deprecated))
#else
# define DEPRECATED__cola2_msgs__srv__DigitalOutput_Request __declspec(deprecated)
#endif

namespace cola2_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct DigitalOutput_Request_
{
  using Type = DigitalOutput_Request_<ContainerAllocator>;

  explicit DigitalOutput_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->digital_output = 0l;
      this->value = 0l;
    }
  }

  explicit DigitalOutput_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->digital_output = 0l;
      this->value = 0l;
    }
  }

  // field types and members
  using _digital_output_type =
    int32_t;
  _digital_output_type digital_output;
  using _value_type =
    int32_t;
  _value_type value;

  // setters for named parameter idiom
  Type & set__digital_output(
    const int32_t & _arg)
  {
    this->digital_output = _arg;
    return *this;
  }
  Type & set__value(
    const int32_t & _arg)
  {
    this->value = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    cola2_msgs::srv::DigitalOutput_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const cola2_msgs::srv::DigitalOutput_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cola2_msgs::srv::DigitalOutput_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cola2_msgs::srv::DigitalOutput_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::srv::DigitalOutput_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::srv::DigitalOutput_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::srv::DigitalOutput_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::srv::DigitalOutput_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cola2_msgs::srv::DigitalOutput_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cola2_msgs::srv::DigitalOutput_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cola2_msgs__srv__DigitalOutput_Request
    std::shared_ptr<cola2_msgs::srv::DigitalOutput_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cola2_msgs__srv__DigitalOutput_Request
    std::shared_ptr<cola2_msgs::srv::DigitalOutput_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DigitalOutput_Request_ & other) const
  {
    if (this->digital_output != other.digital_output) {
      return false;
    }
    if (this->value != other.value) {
      return false;
    }
    return true;
  }
  bool operator!=(const DigitalOutput_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DigitalOutput_Request_

// alias to use template instance with default allocator
using DigitalOutput_Request =
  cola2_msgs::srv::DigitalOutput_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace cola2_msgs


#ifndef _WIN32
# define DEPRECATED__cola2_msgs__srv__DigitalOutput_Response __attribute__((deprecated))
#else
# define DEPRECATED__cola2_msgs__srv__DigitalOutput_Response __declspec(deprecated)
#endif

namespace cola2_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct DigitalOutput_Response_
{
  using Type = DigitalOutput_Response_<ContainerAllocator>;

  explicit DigitalOutput_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit DigitalOutput_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    cola2_msgs::srv::DigitalOutput_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const cola2_msgs::srv::DigitalOutput_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cola2_msgs::srv::DigitalOutput_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cola2_msgs::srv::DigitalOutput_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::srv::DigitalOutput_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::srv::DigitalOutput_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::srv::DigitalOutput_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::srv::DigitalOutput_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cola2_msgs::srv::DigitalOutput_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cola2_msgs::srv::DigitalOutput_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cola2_msgs__srv__DigitalOutput_Response
    std::shared_ptr<cola2_msgs::srv::DigitalOutput_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cola2_msgs__srv__DigitalOutput_Response
    std::shared_ptr<cola2_msgs::srv::DigitalOutput_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DigitalOutput_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const DigitalOutput_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DigitalOutput_Response_

// alias to use template instance with default allocator
using DigitalOutput_Response =
  cola2_msgs::srv::DigitalOutput_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace cola2_msgs

namespace cola2_msgs
{

namespace srv
{

struct DigitalOutput
{
  using Request = cola2_msgs::srv::DigitalOutput_Request;
  using Response = cola2_msgs::srv::DigitalOutput_Response;
};

}  // namespace srv

}  // namespace cola2_msgs

#endif  // COLA2_MSGS__SRV__DETAIL__DIGITAL_OUTPUT__STRUCT_HPP_
