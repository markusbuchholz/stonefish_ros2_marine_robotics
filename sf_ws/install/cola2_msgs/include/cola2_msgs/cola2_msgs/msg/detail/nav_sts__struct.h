// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from cola2_msgs:msg/NavSts.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__NAV_STS__STRUCT_H_
#define COLA2_MSGS__MSG__DETAIL__NAV_STS__STRUCT_H_

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
// Member 'global_position'
// Member 'origin'
#include "cola2_msgs/msg/detail/decimal_lat_lon__struct.h"
// Member 'position'
// Member 'position_variance'
#include "cola2_msgs/msg/detail/ned__struct.h"
// Member 'body_velocity'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'orientation'
// Member 'orientation_rate'
// Member 'orientation_variance'
#include "cola2_msgs/msg/detail/rpy__struct.h"

/// Struct defined in msg/NavSts in the package cola2_msgs.
/**
  * Sent by the navigator
 */
typedef struct cola2_msgs__msg__NavSts
{
  std_msgs__msg__Header header;
  cola2_msgs__msg__DecimalLatLon global_position;
  cola2_msgs__msg__DecimalLatLon origin;
  /// NED position and altitude in metres.
  cola2_msgs__msg__NED position;
  float altitude;
  /// Body velocities in metres/sec.
  geometry_msgs__msg__Point body_velocity;
  /// Orientation and orientation rate are in radians and radians/sec
  cola2_msgs__msg__RPY orientation;
  cola2_msgs__msg__RPY orientation_rate;
  /// Variances for position and orientation
  cola2_msgs__msg__NED position_variance;
  cola2_msgs__msg__RPY orientation_variance;
} cola2_msgs__msg__NavSts;

// Struct for a sequence of cola2_msgs__msg__NavSts.
typedef struct cola2_msgs__msg__NavSts__Sequence
{
  cola2_msgs__msg__NavSts * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cola2_msgs__msg__NavSts__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // COLA2_MSGS__MSG__DETAIL__NAV_STS__STRUCT_H_
