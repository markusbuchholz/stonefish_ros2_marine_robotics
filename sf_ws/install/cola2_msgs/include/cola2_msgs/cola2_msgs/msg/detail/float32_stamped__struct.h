// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from cola2_msgs:msg/Float32Stamped.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__FLOAT32_STAMPED__STRUCT_H_
#define COLA2_MSGS__MSG__DETAIL__FLOAT32_STAMPED__STRUCT_H_

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

/// Struct defined in msg/Float32Stamped in the package cola2_msgs.
/**
  * Single float value with a stamp
 */
typedef struct cola2_msgs__msg__Float32Stamped
{
  std_msgs__msg__Header header;
  float data;
} cola2_msgs__msg__Float32Stamped;

// Struct for a sequence of cola2_msgs__msg__Float32Stamped.
typedef struct cola2_msgs__msg__Float32Stamped__Sequence
{
  cola2_msgs__msg__Float32Stamped * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cola2_msgs__msg__Float32Stamped__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // COLA2_MSGS__MSG__DETAIL__FLOAT32_STAMPED__STRUCT_H_
