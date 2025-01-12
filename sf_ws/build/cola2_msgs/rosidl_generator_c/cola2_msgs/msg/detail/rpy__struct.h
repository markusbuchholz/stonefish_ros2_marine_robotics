// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from cola2_msgs:msg/RPY.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__RPY__STRUCT_H_
#define COLA2_MSGS__MSG__DETAIL__RPY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/RPY in the package cola2_msgs.
/**
  * Orientation or orientation velocity in a frame consistent with the right-handed NED coordinate frame.
 */
typedef struct cola2_msgs__msg__RPY
{
  /// Fixed-axis roll, in radians. Positive clockwise rotation about the X axis, looking forward.
  float roll;
  /// Fixed-axis pitch, in radians. Positive upwards rotation of the vehicle nose about the Y axis.
  float pitch;
  /// Fixed-axis yaw, in radians. Positive clockwise rotation about the Z axis, looking down.
  float yaw;
} cola2_msgs__msg__RPY;

// Struct for a sequence of cola2_msgs__msg__RPY.
typedef struct cola2_msgs__msg__RPY__Sequence
{
  cola2_msgs__msg__RPY * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cola2_msgs__msg__RPY__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // COLA2_MSGS__MSG__DETAIL__RPY__STRUCT_H_
