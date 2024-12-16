// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from stonefish_ros2:msg/BeaconInfo.idl
// generated code does not contain a copyright notice

#ifndef STONEFISH_ROS2__MSG__DETAIL__BEACON_INFO__STRUCT_HPP_
#define STONEFISH_ROS2__MSG__DETAIL__BEACON_INFO__STRUCT_HPP_

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
// Member 'relative_position'
#include "geometry_msgs/msg/detail/point__struct.hpp"
// Member 'local_orientation'
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__stonefish_ros2__msg__BeaconInfo __attribute__((deprecated))
#else
# define DEPRECATED__stonefish_ros2__msg__BeaconInfo __declspec(deprecated)
#endif

namespace stonefish_ros2
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BeaconInfo_
{
  using Type = BeaconInfo_<ContainerAllocator>;

  explicit BeaconInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    relative_position(_init),
    local_orientation(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->beacon_id = 0;
      this->range = 0.0f;
      this->azimuth = 0.0f;
      this->elevation = 0.0f;
      this->local_depth = 0.0f;
    }
  }

  explicit BeaconInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    relative_position(_alloc, _init),
    local_orientation(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->beacon_id = 0;
      this->range = 0.0f;
      this->azimuth = 0.0f;
      this->elevation = 0.0f;
      this->local_depth = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _beacon_id_type =
    uint8_t;
  _beacon_id_type beacon_id;
  using _range_type =
    float;
  _range_type range;
  using _azimuth_type =
    float;
  _azimuth_type azimuth;
  using _elevation_type =
    float;
  _elevation_type elevation;
  using _relative_position_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _relative_position_type relative_position;
  using _local_orientation_type =
    geometry_msgs::msg::Quaternion_<ContainerAllocator>;
  _local_orientation_type local_orientation;
  using _local_depth_type =
    float;
  _local_depth_type local_depth;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__beacon_id(
    const uint8_t & _arg)
  {
    this->beacon_id = _arg;
    return *this;
  }
  Type & set__range(
    const float & _arg)
  {
    this->range = _arg;
    return *this;
  }
  Type & set__azimuth(
    const float & _arg)
  {
    this->azimuth = _arg;
    return *this;
  }
  Type & set__elevation(
    const float & _arg)
  {
    this->elevation = _arg;
    return *this;
  }
  Type & set__relative_position(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->relative_position = _arg;
    return *this;
  }
  Type & set__local_orientation(
    const geometry_msgs::msg::Quaternion_<ContainerAllocator> & _arg)
  {
    this->local_orientation = _arg;
    return *this;
  }
  Type & set__local_depth(
    const float & _arg)
  {
    this->local_depth = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    stonefish_ros2::msg::BeaconInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const stonefish_ros2::msg::BeaconInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<stonefish_ros2::msg::BeaconInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<stonefish_ros2::msg::BeaconInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      stonefish_ros2::msg::BeaconInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<stonefish_ros2::msg::BeaconInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      stonefish_ros2::msg::BeaconInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<stonefish_ros2::msg::BeaconInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<stonefish_ros2::msg::BeaconInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<stonefish_ros2::msg::BeaconInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__stonefish_ros2__msg__BeaconInfo
    std::shared_ptr<stonefish_ros2::msg::BeaconInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__stonefish_ros2__msg__BeaconInfo
    std::shared_ptr<stonefish_ros2::msg::BeaconInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BeaconInfo_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->beacon_id != other.beacon_id) {
      return false;
    }
    if (this->range != other.range) {
      return false;
    }
    if (this->azimuth != other.azimuth) {
      return false;
    }
    if (this->elevation != other.elevation) {
      return false;
    }
    if (this->relative_position != other.relative_position) {
      return false;
    }
    if (this->local_orientation != other.local_orientation) {
      return false;
    }
    if (this->local_depth != other.local_depth) {
      return false;
    }
    return true;
  }
  bool operator!=(const BeaconInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BeaconInfo_

// alias to use template instance with default allocator
using BeaconInfo =
  stonefish_ros2::msg::BeaconInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace stonefish_ros2

#endif  // STONEFISH_ROS2__MSG__DETAIL__BEACON_INFO__STRUCT_HPP_
