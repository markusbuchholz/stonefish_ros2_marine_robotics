// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from cola2_msgs:msg/Setpoints.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__SETPOINTS__STRUCT_H_
#define COLA2_MSGS__MSG__DETAIL__SETPOINTS__STRUCT_H_

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
// Member 'setpoints'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/Setpoints in the package cola2_msgs.
typedef struct cola2_msgs__msg__Setpoints
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__double__Sequence setpoints;
} cola2_msgs__msg__Setpoints;

// Struct for a sequence of cola2_msgs__msg__Setpoints.
typedef struct cola2_msgs__msg__Setpoints__Sequence
{
  cola2_msgs__msg__Setpoints * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cola2_msgs__msg__Setpoints__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // COLA2_MSGS__MSG__DETAIL__SETPOINTS__STRUCT_H_
