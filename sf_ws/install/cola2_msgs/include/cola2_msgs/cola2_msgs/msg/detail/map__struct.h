// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from cola2_msgs:msg/Map.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__MAP__STRUCT_H_
#define COLA2_MSGS__MSG__DETAIL__MAP__STRUCT_H_

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
// Member 'landmark'
#include "cola2_msgs/msg/detail/landmark__struct.h"

/// Struct defined in msg/Map in the package cola2_msgs.
typedef struct cola2_msgs__msg__Map
{
  std_msgs__msg__Header header;
  cola2_msgs__msg__Landmark__Sequence landmark;
} cola2_msgs__msg__Map;

// Struct for a sequence of cola2_msgs__msg__Map.
typedef struct cola2_msgs__msg__Map__Sequence
{
  cola2_msgs__msg__Map * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cola2_msgs__msg__Map__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // COLA2_MSGS__MSG__DETAIL__MAP__STRUCT_H_
