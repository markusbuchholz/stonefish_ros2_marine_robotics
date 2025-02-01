// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from stonefish_ros2:msg/INS.idl
// generated code does not contain a copyright notice

#ifndef STONEFISH_ROS2__MSG__DETAIL__INS__STRUCT_H_
#define STONEFISH_ROS2__MSG__DETAIL__INS__STRUCT_H_

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
// Member 'pose'
// Member 'pose_variance'
#include "stonefish_ros2/msg/detail/ned_pose__struct.h"
// Member 'body_velocity'
// Member 'rpy_rate'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/INS in the package stonefish_ros2.
typedef struct stonefish_ros2__msg__INS
{
  std_msgs__msg__Header header;
  /// Global position of robot
  double latitude;
  double longitude;
  /// Global position of NED origin
  double origin_latitude;
  double origin_longitude;
  /// Robot pose in NED
  stonefish_ros2__msg__NEDPose pose;
  stonefish_ros2__msg__NEDPose pose_variance;
  double altitude;
  /// Robot body velocity
  geometry_msgs__msg__Vector3 body_velocity;
  geometry_msgs__msg__Vector3 rpy_rate;
} stonefish_ros2__msg__INS;

// Struct for a sequence of stonefish_ros2__msg__INS.
typedef struct stonefish_ros2__msg__INS__Sequence
{
  stonefish_ros2__msg__INS * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stonefish_ros2__msg__INS__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // STONEFISH_ROS2__MSG__DETAIL__INS__STRUCT_H_
