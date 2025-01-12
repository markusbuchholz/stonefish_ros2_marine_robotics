// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from cola2_msgs:msg/Landmark.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__LANDMARK__STRUCT_H_
#define COLA2_MSGS__MSG__DETAIL__LANDMARK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'last_update'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'landmark_id'
#include "rosidl_runtime_c/string.h"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_with_covariance__struct.h"

/// Struct defined in msg/Landmark in the package cola2_msgs.
typedef struct cola2_msgs__msg__Landmark
{
  builtin_interfaces__msg__Time last_update;
  rosidl_runtime_c__String landmark_id;
  geometry_msgs__msg__PoseWithCovariance pose;
} cola2_msgs__msg__Landmark;

// Struct for a sequence of cola2_msgs__msg__Landmark.
typedef struct cola2_msgs__msg__Landmark__Sequence
{
  cola2_msgs__msg__Landmark * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cola2_msgs__msg__Landmark__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // COLA2_MSGS__MSG__DETAIL__LANDMARK__STRUCT_H_
