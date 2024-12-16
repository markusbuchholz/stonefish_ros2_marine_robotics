// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from cola2_msgs:msg/Bool6Axis.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__BOOL6_AXIS__STRUCT_H_
#define COLA2_MSGS__MSG__DETAIL__BOOL6_AXIS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Bool6Axis in the package cola2_msgs.
typedef struct cola2_msgs__msg__Bool6Axis
{
  bool x;
  bool y;
  bool z;
  bool roll;
  bool pitch;
  bool yaw;
} cola2_msgs__msg__Bool6Axis;

// Struct for a sequence of cola2_msgs__msg__Bool6Axis.
typedef struct cola2_msgs__msg__Bool6Axis__Sequence
{
  cola2_msgs__msg__Bool6Axis * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cola2_msgs__msg__Bool6Axis__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // COLA2_MSGS__MSG__DETAIL__BOOL6_AXIS__STRUCT_H_
