// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from cola2_msgs:msg/DecimalLatLon.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__DECIMAL_LAT_LON__STRUCT_HPP_
#define COLA2_MSGS__MSG__DETAIL__DECIMAL_LAT_LON__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__cola2_msgs__msg__DecimalLatLon __attribute__((deprecated))
#else
# define DEPRECATED__cola2_msgs__msg__DecimalLatLon __declspec(deprecated)
#endif

namespace cola2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DecimalLatLon_
{
  using Type = DecimalLatLon_<ContainerAllocator>;

  explicit DecimalLatLon_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->latitude = 0.0;
      this->longitude = 0.0;
    }
  }

  explicit DecimalLatLon_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->latitude = 0.0;
      this->longitude = 0.0;
    }
  }

  // field types and members
  using _latitude_type =
    double;
  _latitude_type latitude;
  using _longitude_type =
    double;
  _longitude_type longitude;

  // setters for named parameter idiom
  Type & set__latitude(
    const double & _arg)
  {
    this->latitude = _arg;
    return *this;
  }
  Type & set__longitude(
    const double & _arg)
  {
    this->longitude = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    cola2_msgs::msg::DecimalLatLon_<ContainerAllocator> *;
  using ConstRawPtr =
    const cola2_msgs::msg::DecimalLatLon_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cola2_msgs::msg::DecimalLatLon_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cola2_msgs::msg::DecimalLatLon_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::msg::DecimalLatLon_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::msg::DecimalLatLon_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::msg::DecimalLatLon_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::msg::DecimalLatLon_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cola2_msgs::msg::DecimalLatLon_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cola2_msgs::msg::DecimalLatLon_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cola2_msgs__msg__DecimalLatLon
    std::shared_ptr<cola2_msgs::msg::DecimalLatLon_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cola2_msgs__msg__DecimalLatLon
    std::shared_ptr<cola2_msgs::msg::DecimalLatLon_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DecimalLatLon_ & other) const
  {
    if (this->latitude != other.latitude) {
      return false;
    }
    if (this->longitude != other.longitude) {
      return false;
    }
    return true;
  }
  bool operator!=(const DecimalLatLon_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DecimalLatLon_

// alias to use template instance with default allocator
using DecimalLatLon =
  cola2_msgs::msg::DecimalLatLon_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace cola2_msgs

#endif  // COLA2_MSGS__MSG__DETAIL__DECIMAL_LAT_LON__STRUCT_HPP_
