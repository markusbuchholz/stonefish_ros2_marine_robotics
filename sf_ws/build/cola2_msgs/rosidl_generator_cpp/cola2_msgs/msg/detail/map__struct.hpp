// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from cola2_msgs:msg/Map.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__MAP__STRUCT_HPP_
#define COLA2_MSGS__MSG__DETAIL__MAP__STRUCT_HPP_

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
// Member 'landmark'
#include "cola2_msgs/msg/detail/landmark__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__cola2_msgs__msg__Map __attribute__((deprecated))
#else
# define DEPRECATED__cola2_msgs__msg__Map __declspec(deprecated)
#endif

namespace cola2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Map_
{
  using Type = Map_<ContainerAllocator>;

  explicit Map_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit Map_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _landmark_type =
    std::vector<cola2_msgs::msg::Landmark_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<cola2_msgs::msg::Landmark_<ContainerAllocator>>>;
  _landmark_type landmark;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__landmark(
    const std::vector<cola2_msgs::msg::Landmark_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<cola2_msgs::msg::Landmark_<ContainerAllocator>>> & _arg)
  {
    this->landmark = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    cola2_msgs::msg::Map_<ContainerAllocator> *;
  using ConstRawPtr =
    const cola2_msgs::msg::Map_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cola2_msgs::msg::Map_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cola2_msgs::msg::Map_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::msg::Map_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::msg::Map_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::msg::Map_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::msg::Map_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cola2_msgs::msg::Map_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cola2_msgs::msg::Map_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cola2_msgs__msg__Map
    std::shared_ptr<cola2_msgs::msg::Map_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cola2_msgs__msg__Map
    std::shared_ptr<cola2_msgs::msg::Map_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Map_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->landmark != other.landmark) {
      return false;
    }
    return true;
  }
  bool operator!=(const Map_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Map_

// alias to use template instance with default allocator
using Map =
  cola2_msgs::msg::Map_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace cola2_msgs

#endif  // COLA2_MSGS__MSG__DETAIL__MAP__STRUCT_HPP_
