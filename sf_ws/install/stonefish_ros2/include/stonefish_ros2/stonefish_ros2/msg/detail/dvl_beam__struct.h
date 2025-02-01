// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from stonefish_ros2:msg/DVLBeam.idl
// generated code does not contain a copyright notice

#ifndef STONEFISH_ROS2__MSG__DETAIL__DVL_BEAM__STRUCT_H_
#define STONEFISH_ROS2__MSG__DETAIL__DVL_BEAM__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in msg/DVLBeam in the package stonefish_ros2.
/**
  * Extracted from UUV Simulator:
  *   https://github.com/uuvsimulator/uuv_simulator
 */
typedef struct stonefish_ros2__msg__DVLBeam
{
  /// measured range or < 0 if invalid
  double range;
  double range_covariance;
  /// measured velocity of corr. beam
  double velocity;
  double velocity_covariance;
  /// Beam link pose wrt DVL link frame
  geometry_msgs__msg__PoseStamped pose;
} stonefish_ros2__msg__DVLBeam;

// Struct for a sequence of stonefish_ros2__msg__DVLBeam.
typedef struct stonefish_ros2__msg__DVLBeam__Sequence
{
  stonefish_ros2__msg__DVLBeam * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stonefish_ros2__msg__DVLBeam__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // STONEFISH_ROS2__MSG__DETAIL__DVL_BEAM__STRUCT_H_
