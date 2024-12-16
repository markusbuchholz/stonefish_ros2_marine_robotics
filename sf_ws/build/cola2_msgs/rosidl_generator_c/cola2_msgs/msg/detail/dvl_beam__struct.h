// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from cola2_msgs:msg/DVLBeam.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__DVL_BEAM__STRUCT_H_
#define COLA2_MSGS__MSG__DETAIL__DVL_BEAM__STRUCT_H_

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

/// Struct defined in msg/DVLBeam in the package cola2_msgs.
/**
  * Extracted from UUV Simulator:
  *   https://github.com/uuvsimulator/uuv_simulator
 */
typedef struct cola2_msgs__msg__DVLBeam
{
  /// measured range or < 0 if invalid
  double range;
  double range_covariance;
  /// measured velocity of corr. beam
  double velocity;
  double velocity_covariance;
  /// Beam link pose wrt DVL link frame
  geometry_msgs__msg__PoseStamped pose;
} cola2_msgs__msg__DVLBeam;

// Struct for a sequence of cola2_msgs__msg__DVLBeam.
typedef struct cola2_msgs__msg__DVLBeam__Sequence
{
  cola2_msgs__msg__DVLBeam * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cola2_msgs__msg__DVLBeam__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // COLA2_MSGS__MSG__DETAIL__DVL_BEAM__STRUCT_H_
