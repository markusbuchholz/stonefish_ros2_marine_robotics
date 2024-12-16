// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from stonefish_ros2:msg/INS.idl
// generated code does not contain a copyright notice

#ifndef STONEFISH_ROS2__MSG__DETAIL__INS__STRUCT_HPP_
#define STONEFISH_ROS2__MSG__DETAIL__INS__STRUCT_HPP_

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
// Member 'pose'
// Member 'pose_variance'
#include "stonefish_ros2/msg/detail/ned_pose__struct.hpp"
// Member 'body_velocity'
// Member 'rpy_rate'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__stonefish_ros2__msg__INS __attribute__((deprecated))
#else
# define DEPRECATED__stonefish_ros2__msg__INS __declspec(deprecated)
#endif

namespace stonefish_ros2
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct INS_
{
  using Type = INS_<ContainerAllocator>;

  explicit INS_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    pose(_init),
    pose_variance(_init),
    body_velocity(_init),
    rpy_rate(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->latitude = 0.0;
      this->longitude = 0.0;
      this->origin_latitude = 0.0;
      this->origin_longitude = 0.0;
      this->altitude = 0.0;
    }
  }

  explicit INS_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    pose(_alloc, _init),
    pose_variance(_alloc, _init),
    body_velocity(_alloc, _init),
    rpy_rate(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->latitude = 0.0;
      this->longitude = 0.0;
      this->origin_latitude = 0.0;
      this->origin_longitude = 0.0;
      this->altitude = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _latitude_type =
    double;
  _latitude_type latitude;
  using _longitude_type =
    double;
  _longitude_type longitude;
  using _origin_latitude_type =
    double;
  _origin_latitude_type origin_latitude;
  using _origin_longitude_type =
    double;
  _origin_longitude_type origin_longitude;
  using _pose_type =
    stonefish_ros2::msg::NEDPose_<ContainerAllocator>;
  _pose_type pose;
  using _pose_variance_type =
    stonefish_ros2::msg::NEDPose_<ContainerAllocator>;
  _pose_variance_type pose_variance;
  using _altitude_type =
    double;
  _altitude_type altitude;
  using _body_velocity_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _body_velocity_type body_velocity;
  using _rpy_rate_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _rpy_rate_type rpy_rate;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
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
  Type & set__origin_latitude(
    const double & _arg)
  {
    this->origin_latitude = _arg;
    return *this;
  }
  Type & set__origin_longitude(
    const double & _arg)
  {
    this->origin_longitude = _arg;
    return *this;
  }
  Type & set__pose(
    const stonefish_ros2::msg::NEDPose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__pose_variance(
    const stonefish_ros2::msg::NEDPose_<ContainerAllocator> & _arg)
  {
    this->pose_variance = _arg;
    return *this;
  }
  Type & set__altitude(
    const double & _arg)
  {
    this->altitude = _arg;
    return *this;
  }
  Type & set__body_velocity(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->body_velocity = _arg;
    return *this;
  }
  Type & set__rpy_rate(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->rpy_rate = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    stonefish_ros2::msg::INS_<ContainerAllocator> *;
  using ConstRawPtr =
    const stonefish_ros2::msg::INS_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<stonefish_ros2::msg::INS_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<stonefish_ros2::msg::INS_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      stonefish_ros2::msg::INS_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<stonefish_ros2::msg::INS_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      stonefish_ros2::msg::INS_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<stonefish_ros2::msg::INS_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<stonefish_ros2::msg::INS_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<stonefish_ros2::msg::INS_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__stonefish_ros2__msg__INS
    std::shared_ptr<stonefish_ros2::msg::INS_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__stonefish_ros2__msg__INS
    std::shared_ptr<stonefish_ros2::msg::INS_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const INS_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->latitude != other.latitude) {
      return false;
    }
    if (this->longitude != other.longitude) {
      return false;
    }
    if (this->origin_latitude != other.origin_latitude) {
      return false;
    }
    if (this->origin_longitude != other.origin_longitude) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    if (this->pose_variance != other.pose_variance) {
      return false;
    }
    if (this->altitude != other.altitude) {
      return false;
    }
    if (this->body_velocity != other.body_velocity) {
      return false;
    }
    if (this->rpy_rate != other.rpy_rate) {
      return false;
    }
    return true;
  }
  bool operator!=(const INS_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct INS_

// alias to use template instance with default allocator
using INS =
  stonefish_ros2::msg::INS_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace stonefish_ros2

#endif  // STONEFISH_ROS2__MSG__DETAIL__INS__STRUCT_HPP_
