// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from cola2_msgs:msg/NavSts.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__NAV_STS__STRUCT_HPP_
#define COLA2_MSGS__MSG__DETAIL__NAV_STS__STRUCT_HPP_

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
// Member 'global_position'
// Member 'origin'
#include "cola2_msgs/msg/detail/decimal_lat_lon__struct.hpp"
// Member 'position'
// Member 'position_variance'
#include "cola2_msgs/msg/detail/ned__struct.hpp"
// Member 'body_velocity'
#include "geometry_msgs/msg/detail/point__struct.hpp"
// Member 'orientation'
// Member 'orientation_rate'
// Member 'orientation_variance'
#include "cola2_msgs/msg/detail/rpy__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__cola2_msgs__msg__NavSts __attribute__((deprecated))
#else
# define DEPRECATED__cola2_msgs__msg__NavSts __declspec(deprecated)
#endif

namespace cola2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct NavSts_
{
  using Type = NavSts_<ContainerAllocator>;

  explicit NavSts_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    global_position(_init),
    origin(_init),
    position(_init),
    body_velocity(_init),
    orientation(_init),
    orientation_rate(_init),
    position_variance(_init),
    orientation_variance(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->altitude = 0.0f;
    }
  }

  explicit NavSts_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    global_position(_alloc, _init),
    origin(_alloc, _init),
    position(_alloc, _init),
    body_velocity(_alloc, _init),
    orientation(_alloc, _init),
    orientation_rate(_alloc, _init),
    position_variance(_alloc, _init),
    orientation_variance(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->altitude = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _global_position_type =
    cola2_msgs::msg::DecimalLatLon_<ContainerAllocator>;
  _global_position_type global_position;
  using _origin_type =
    cola2_msgs::msg::DecimalLatLon_<ContainerAllocator>;
  _origin_type origin;
  using _position_type =
    cola2_msgs::msg::NED_<ContainerAllocator>;
  _position_type position;
  using _altitude_type =
    float;
  _altitude_type altitude;
  using _body_velocity_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _body_velocity_type body_velocity;
  using _orientation_type =
    cola2_msgs::msg::RPY_<ContainerAllocator>;
  _orientation_type orientation;
  using _orientation_rate_type =
    cola2_msgs::msg::RPY_<ContainerAllocator>;
  _orientation_rate_type orientation_rate;
  using _position_variance_type =
    cola2_msgs::msg::NED_<ContainerAllocator>;
  _position_variance_type position_variance;
  using _orientation_variance_type =
    cola2_msgs::msg::RPY_<ContainerAllocator>;
  _orientation_variance_type orientation_variance;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__global_position(
    const cola2_msgs::msg::DecimalLatLon_<ContainerAllocator> & _arg)
  {
    this->global_position = _arg;
    return *this;
  }
  Type & set__origin(
    const cola2_msgs::msg::DecimalLatLon_<ContainerAllocator> & _arg)
  {
    this->origin = _arg;
    return *this;
  }
  Type & set__position(
    const cola2_msgs::msg::NED_<ContainerAllocator> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__altitude(
    const float & _arg)
  {
    this->altitude = _arg;
    return *this;
  }
  Type & set__body_velocity(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->body_velocity = _arg;
    return *this;
  }
  Type & set__orientation(
    const cola2_msgs::msg::RPY_<ContainerAllocator> & _arg)
  {
    this->orientation = _arg;
    return *this;
  }
  Type & set__orientation_rate(
    const cola2_msgs::msg::RPY_<ContainerAllocator> & _arg)
  {
    this->orientation_rate = _arg;
    return *this;
  }
  Type & set__position_variance(
    const cola2_msgs::msg::NED_<ContainerAllocator> & _arg)
  {
    this->position_variance = _arg;
    return *this;
  }
  Type & set__orientation_variance(
    const cola2_msgs::msg::RPY_<ContainerAllocator> & _arg)
  {
    this->orientation_variance = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    cola2_msgs::msg::NavSts_<ContainerAllocator> *;
  using ConstRawPtr =
    const cola2_msgs::msg::NavSts_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cola2_msgs::msg::NavSts_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cola2_msgs::msg::NavSts_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::msg::NavSts_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::msg::NavSts_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::msg::NavSts_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::msg::NavSts_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cola2_msgs::msg::NavSts_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cola2_msgs::msg::NavSts_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cola2_msgs__msg__NavSts
    std::shared_ptr<cola2_msgs::msg::NavSts_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cola2_msgs__msg__NavSts
    std::shared_ptr<cola2_msgs::msg::NavSts_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NavSts_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->global_position != other.global_position) {
      return false;
    }
    if (this->origin != other.origin) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->altitude != other.altitude) {
      return false;
    }
    if (this->body_velocity != other.body_velocity) {
      return false;
    }
    if (this->orientation != other.orientation) {
      return false;
    }
    if (this->orientation_rate != other.orientation_rate) {
      return false;
    }
    if (this->position_variance != other.position_variance) {
      return false;
    }
    if (this->orientation_variance != other.orientation_variance) {
      return false;
    }
    return true;
  }
  bool operator!=(const NavSts_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NavSts_

// alias to use template instance with default allocator
using NavSts =
  cola2_msgs::msg::NavSts_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace cola2_msgs

#endif  // COLA2_MSGS__MSG__DETAIL__NAV_STS__STRUCT_HPP_
