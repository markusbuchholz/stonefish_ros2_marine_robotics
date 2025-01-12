// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from cola2_msgs:msg/NED.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__NED__STRUCT_H_
#define COLA2_MSGS__MSG__DETAIL__NED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/NED in the package cola2_msgs.
/**
  * Position or velocity in the right-handed NED coordinate frame.
 */
typedef struct cola2_msgs__msg__NED
{
  /// metres north, positive towards true north.
  double north;
  /// metres east, positive towards true east.
  double east;
  /// metres down, positive towards the centre of the earth.
  double depth;
} cola2_msgs__msg__NED;

// Struct for a sequence of cola2_msgs__msg__NED.
typedef struct cola2_msgs__msg__NED__Sequence
{
  cola2_msgs__msg__NED * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cola2_msgs__msg__NED__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // COLA2_MSGS__MSG__DETAIL__NED__STRUCT_H_
