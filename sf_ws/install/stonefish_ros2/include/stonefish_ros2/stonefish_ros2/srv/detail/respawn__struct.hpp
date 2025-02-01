// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from stonefish_ros2:srv/Respawn.idl
// generated code does not contain a copyright notice

#ifndef STONEFISH_ROS2__SRV__DETAIL__RESPAWN__STRUCT_HPP_
#define STONEFISH_ROS2__SRV__DETAIL__RESPAWN__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'origin'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__stonefish_ros2__srv__Respawn_Request __attribute__((deprecated))
#else
# define DEPRECATED__stonefish_ros2__srv__Respawn_Request __declspec(deprecated)
#endif

namespace stonefish_ros2
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Respawn_Request_
{
  using Type = Respawn_Request_<ContainerAllocator>;

  explicit Respawn_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : origin(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
    }
  }

  explicit Respawn_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : name(_alloc),
    origin(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
    }
  }

  // field types and members
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;
  using _origin_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _origin_type origin;

  // setters for named parameter idiom
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__origin(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->origin = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    stonefish_ros2::srv::Respawn_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const stonefish_ros2::srv::Respawn_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<stonefish_ros2::srv::Respawn_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<stonefish_ros2::srv::Respawn_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      stonefish_ros2::srv::Respawn_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<stonefish_ros2::srv::Respawn_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      stonefish_ros2::srv::Respawn_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<stonefish_ros2::srv::Respawn_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<stonefish_ros2::srv::Respawn_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<stonefish_ros2::srv::Respawn_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__stonefish_ros2__srv__Respawn_Request
    std::shared_ptr<stonefish_ros2::srv::Respawn_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__stonefish_ros2__srv__Respawn_Request
    std::shared_ptr<stonefish_ros2::srv::Respawn_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Respawn_Request_ & other) const
  {
    if (this->name != other.name) {
      return false;
    }
    if (this->origin != other.origin) {
      return false;
    }
    return true;
  }
  bool operator!=(const Respawn_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Respawn_Request_

// alias to use template instance with default allocator
using Respawn_Request =
  stonefish_ros2::srv::Respawn_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace stonefish_ros2


#ifndef _WIN32
# define DEPRECATED__stonefish_ros2__srv__Respawn_Response __attribute__((deprecated))
#else
# define DEPRECATED__stonefish_ros2__srv__Respawn_Response __declspec(deprecated)
#endif

namespace stonefish_ros2
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Respawn_Response_
{
  using Type = Respawn_Response_<ContainerAllocator>;

  explicit Respawn_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit Respawn_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    stonefish_ros2::srv::Respawn_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const stonefish_ros2::srv::Respawn_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<stonefish_ros2::srv::Respawn_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<stonefish_ros2::srv::Respawn_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      stonefish_ros2::srv::Respawn_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<stonefish_ros2::srv::Respawn_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      stonefish_ros2::srv::Respawn_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<stonefish_ros2::srv::Respawn_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<stonefish_ros2::srv::Respawn_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<stonefish_ros2::srv::Respawn_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__stonefish_ros2__srv__Respawn_Response
    std::shared_ptr<stonefish_ros2::srv::Respawn_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__stonefish_ros2__srv__Respawn_Response
    std::shared_ptr<stonefish_ros2::srv::Respawn_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Respawn_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const Respawn_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Respawn_Response_

// alias to use template instance with default allocator
using Respawn_Response =
  stonefish_ros2::srv::Respawn_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace stonefish_ros2

namespace stonefish_ros2
{

namespace srv
{

struct Respawn
{
  using Request = stonefish_ros2::srv::Respawn_Request;
  using Response = stonefish_ros2::srv::Respawn_Response;
};

}  // namespace srv

}  // namespace stonefish_ros2

#endif  // STONEFISH_ROS2__SRV__DETAIL__RESPAWN__STRUCT_HPP_
