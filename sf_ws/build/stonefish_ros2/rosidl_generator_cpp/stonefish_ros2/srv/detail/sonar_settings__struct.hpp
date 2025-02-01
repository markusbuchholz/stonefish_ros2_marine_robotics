// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from stonefish_ros2:srv/SonarSettings.idl
// generated code does not contain a copyright notice

#ifndef STONEFISH_ROS2__SRV__DETAIL__SONAR_SETTINGS__STRUCT_HPP_
#define STONEFISH_ROS2__SRV__DETAIL__SONAR_SETTINGS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__stonefish_ros2__srv__SonarSettings_Request __attribute__((deprecated))
#else
# define DEPRECATED__stonefish_ros2__srv__SonarSettings_Request __declspec(deprecated)
#endif

namespace stonefish_ros2
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SonarSettings_Request_
{
  using Type = SonarSettings_Request_<ContainerAllocator>;

  explicit SonarSettings_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->range_min = 0.0;
      this->range_max = 0.0;
      this->gain = 0.0;
    }
  }

  explicit SonarSettings_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->range_min = 0.0;
      this->range_max = 0.0;
      this->gain = 0.0;
    }
  }

  // field types and members
  using _range_min_type =
    double;
  _range_min_type range_min;
  using _range_max_type =
    double;
  _range_max_type range_max;
  using _gain_type =
    double;
  _gain_type gain;

  // setters for named parameter idiom
  Type & set__range_min(
    const double & _arg)
  {
    this->range_min = _arg;
    return *this;
  }
  Type & set__range_max(
    const double & _arg)
  {
    this->range_max = _arg;
    return *this;
  }
  Type & set__gain(
    const double & _arg)
  {
    this->gain = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    stonefish_ros2::srv::SonarSettings_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const stonefish_ros2::srv::SonarSettings_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<stonefish_ros2::srv::SonarSettings_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<stonefish_ros2::srv::SonarSettings_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      stonefish_ros2::srv::SonarSettings_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<stonefish_ros2::srv::SonarSettings_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      stonefish_ros2::srv::SonarSettings_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<stonefish_ros2::srv::SonarSettings_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<stonefish_ros2::srv::SonarSettings_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<stonefish_ros2::srv::SonarSettings_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__stonefish_ros2__srv__SonarSettings_Request
    std::shared_ptr<stonefish_ros2::srv::SonarSettings_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__stonefish_ros2__srv__SonarSettings_Request
    std::shared_ptr<stonefish_ros2::srv::SonarSettings_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SonarSettings_Request_ & other) const
  {
    if (this->range_min != other.range_min) {
      return false;
    }
    if (this->range_max != other.range_max) {
      return false;
    }
    if (this->gain != other.gain) {
      return false;
    }
    return true;
  }
  bool operator!=(const SonarSettings_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SonarSettings_Request_

// alias to use template instance with default allocator
using SonarSettings_Request =
  stonefish_ros2::srv::SonarSettings_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace stonefish_ros2


#ifndef _WIN32
# define DEPRECATED__stonefish_ros2__srv__SonarSettings_Response __attribute__((deprecated))
#else
# define DEPRECATED__stonefish_ros2__srv__SonarSettings_Response __declspec(deprecated)
#endif

namespace stonefish_ros2
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SonarSettings_Response_
{
  using Type = SonarSettings_Response_<ContainerAllocator>;

  explicit SonarSettings_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit SonarSettings_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    stonefish_ros2::srv::SonarSettings_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const stonefish_ros2::srv::SonarSettings_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<stonefish_ros2::srv::SonarSettings_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<stonefish_ros2::srv::SonarSettings_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      stonefish_ros2::srv::SonarSettings_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<stonefish_ros2::srv::SonarSettings_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      stonefish_ros2::srv::SonarSettings_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<stonefish_ros2::srv::SonarSettings_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<stonefish_ros2::srv::SonarSettings_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<stonefish_ros2::srv::SonarSettings_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__stonefish_ros2__srv__SonarSettings_Response
    std::shared_ptr<stonefish_ros2::srv::SonarSettings_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__stonefish_ros2__srv__SonarSettings_Response
    std::shared_ptr<stonefish_ros2::srv::SonarSettings_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SonarSettings_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const SonarSettings_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SonarSettings_Response_

// alias to use template instance with default allocator
using SonarSettings_Response =
  stonefish_ros2::srv::SonarSettings_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace stonefish_ros2

namespace stonefish_ros2
{

namespace srv
{

struct SonarSettings
{
  using Request = stonefish_ros2::srv::SonarSettings_Request;
  using Response = stonefish_ros2::srv::SonarSettings_Response;
};

}  // namespace srv

}  // namespace stonefish_ros2

#endif  // STONEFISH_ROS2__SRV__DETAIL__SONAR_SETTINGS__STRUCT_HPP_
