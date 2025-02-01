// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from cola2_msgs:msg/WorldWaypointReq.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__WORLD_WAYPOINT_REQ__STRUCT_HPP_
#define COLA2_MSGS__MSG__DETAIL__WORLD_WAYPOINT_REQ__STRUCT_HPP_

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
// Member 'goal'
#include "cola2_msgs/msg/detail/goal_descriptor__struct.hpp"
// Member 'position'
#include "cola2_msgs/msg/detail/ned__struct.hpp"
// Member 'orientation'
// Member 'orientation_tolerance'
#include "cola2_msgs/msg/detail/rpy__struct.hpp"
// Member 'disable_axis'
#include "cola2_msgs/msg/detail/bool6_axis__struct.hpp"
// Member 'position_tolerance'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__cola2_msgs__msg__WorldWaypointReq __attribute__((deprecated))
#else
# define DEPRECATED__cola2_msgs__msg__WorldWaypointReq __declspec(deprecated)
#endif

namespace cola2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct WorldWaypointReq_
{
  using Type = WorldWaypointReq_<ContainerAllocator>;

  explicit WorldWaypointReq_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    goal(_init),
    position(_init),
    orientation(_init),
    disable_axis(_init),
    position_tolerance(_init),
    orientation_tolerance(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->altitude_mode = false;
      this->altitude = 0.0f;
    }
  }

  explicit WorldWaypointReq_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    goal(_alloc, _init),
    position(_alloc, _init),
    orientation(_alloc, _init),
    disable_axis(_alloc, _init),
    position_tolerance(_alloc, _init),
    orientation_tolerance(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->altitude_mode = false;
      this->altitude = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _goal_type =
    cola2_msgs::msg::GoalDescriptor_<ContainerAllocator>;
  _goal_type goal;
  using _altitude_mode_type =
    bool;
  _altitude_mode_type altitude_mode;
  using _position_type =
    cola2_msgs::msg::NED_<ContainerAllocator>;
  _position_type position;
  using _altitude_type =
    float;
  _altitude_type altitude;
  using _orientation_type =
    cola2_msgs::msg::RPY_<ContainerAllocator>;
  _orientation_type orientation;
  using _disable_axis_type =
    cola2_msgs::msg::Bool6Axis_<ContainerAllocator>;
  _disable_axis_type disable_axis;
  using _position_tolerance_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _position_tolerance_type position_tolerance;
  using _orientation_tolerance_type =
    cola2_msgs::msg::RPY_<ContainerAllocator>;
  _orientation_tolerance_type orientation_tolerance;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__goal(
    const cola2_msgs::msg::GoalDescriptor_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }
  Type & set__altitude_mode(
    const bool & _arg)
  {
    this->altitude_mode = _arg;
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
  Type & set__orientation(
    const cola2_msgs::msg::RPY_<ContainerAllocator> & _arg)
  {
    this->orientation = _arg;
    return *this;
  }
  Type & set__disable_axis(
    const cola2_msgs::msg::Bool6Axis_<ContainerAllocator> & _arg)
  {
    this->disable_axis = _arg;
    return *this;
  }
  Type & set__position_tolerance(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->position_tolerance = _arg;
    return *this;
  }
  Type & set__orientation_tolerance(
    const cola2_msgs::msg::RPY_<ContainerAllocator> & _arg)
  {
    this->orientation_tolerance = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    cola2_msgs::msg::WorldWaypointReq_<ContainerAllocator> *;
  using ConstRawPtr =
    const cola2_msgs::msg::WorldWaypointReq_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cola2_msgs::msg::WorldWaypointReq_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cola2_msgs::msg::WorldWaypointReq_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::msg::WorldWaypointReq_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::msg::WorldWaypointReq_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::msg::WorldWaypointReq_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::msg::WorldWaypointReq_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cola2_msgs::msg::WorldWaypointReq_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cola2_msgs::msg::WorldWaypointReq_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cola2_msgs__msg__WorldWaypointReq
    std::shared_ptr<cola2_msgs::msg::WorldWaypointReq_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cola2_msgs__msg__WorldWaypointReq
    std::shared_ptr<cola2_msgs::msg::WorldWaypointReq_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WorldWaypointReq_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    if (this->altitude_mode != other.altitude_mode) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->altitude != other.altitude) {
      return false;
    }
    if (this->orientation != other.orientation) {
      return false;
    }
    if (this->disable_axis != other.disable_axis) {
      return false;
    }
    if (this->position_tolerance != other.position_tolerance) {
      return false;
    }
    if (this->orientation_tolerance != other.orientation_tolerance) {
      return false;
    }
    return true;
  }
  bool operator!=(const WorldWaypointReq_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WorldWaypointReq_

// alias to use template instance with default allocator
using WorldWaypointReq =
  cola2_msgs::msg::WorldWaypointReq_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace cola2_msgs

#endif  // COLA2_MSGS__MSG__DETAIL__WORLD_WAYPOINT_REQ__STRUCT_HPP_
