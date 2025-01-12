// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from stonefish_ros2:msg/BeaconInfo.idl
// generated code does not contain a copyright notice

#ifndef STONEFISH_ROS2__MSG__DETAIL__BEACON_INFO__STRUCT_H_
#define STONEFISH_ROS2__MSG__DETAIL__BEACON_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'relative_position'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'local_orientation'
#include "geometry_msgs/msg/detail/quaternion__struct.h"

/// Struct defined in msg/BeaconInfo in the package stonefish_ros2.
typedef struct stonefish_ros2__msg__BeaconInfo
{
  std_msgs__msg__Header header;
  uint8_t beacon_id;
  float range;
  float azimuth;
  float elevation;
  geometry_msgs__msg__Point relative_position;
  geometry_msgs__msg__Quaternion local_orientation;
  float local_depth;
} stonefish_ros2__msg__BeaconInfo;

// Struct for a sequence of stonefish_ros2__msg__BeaconInfo.
typedef struct stonefish_ros2__msg__BeaconInfo__Sequence
{
  stonefish_ros2__msg__BeaconInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stonefish_ros2__msg__BeaconInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // STONEFISH_ROS2__MSG__DETAIL__BEACON_INFO__STRUCT_H_
