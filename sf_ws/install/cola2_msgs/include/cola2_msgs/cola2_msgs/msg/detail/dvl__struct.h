// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from cola2_msgs:msg/DVL.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__DVL__STRUCT_H_
#define COLA2_MSGS__MSG__DETAIL__DVL__STRUCT_H_

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
// Member 'velocity'
#include "geometry_msgs/msg/detail/vector3__struct.h"
// Member 'beams'
#include "cola2_msgs/msg/detail/dvl_beam__struct.h"

/// Struct defined in msg/DVL in the package cola2_msgs.
/**
  * Extracted from UUV Simulator:
  *   https://github.com/uuvsimulator/uuv_simulator
 */
typedef struct cola2_msgs__msg__DVL
{
  /// This is a message to hold data from a DVL sensor (Doppler Velocity Log).
  ///
  /// Distances are in [m], velocities in [m/s]
  ///
  /// If the covariance is known, it should be filled.
  /// If it is unknown, it should be set to all zeros.
  /// If a measurement was invalid, its covariance should be set to -1 so it can be
  /// disregarded.
  ///
  /// DVLBeams are optional. If they are set they contain individual ranges and 1D
  /// doppler velocity estimates orthogonal to the ray.
  std_msgs__msg__Header header;
  /// Measured velocity
  geometry_msgs__msg__Vector3 velocity;
  /// Row major, xyz axes
  double velocity_covariance[9];
  /// Altitude of the vehicle
  double altitude;
  cola2_msgs__msg__DVLBeam__Sequence beams;
} cola2_msgs__msg__DVL;

// Struct for a sequence of cola2_msgs__msg__DVL.
typedef struct cola2_msgs__msg__DVL__Sequence
{
  cola2_msgs__msg__DVL * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cola2_msgs__msg__DVL__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // COLA2_MSGS__MSG__DETAIL__DVL__STRUCT_H_
