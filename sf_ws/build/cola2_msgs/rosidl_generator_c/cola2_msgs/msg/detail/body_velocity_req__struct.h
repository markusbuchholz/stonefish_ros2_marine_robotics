// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from cola2_msgs:msg/BodyVelocityReq.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__BODY_VELOCITY_REQ__STRUCT_H_
#define COLA2_MSGS__MSG__DETAIL__BODY_VELOCITY_REQ__STRUCT_H_

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
// Member 'goal'
#include "cola2_msgs/msg/detail/goal_descriptor__struct.h"
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__struct.h"
// Member 'disable_axis'
#include "cola2_msgs/msg/detail/bool6_axis__struct.h"

/// Struct defined in msg/BodyVelocityReq in the package cola2_msgs.
/**
  * Body frame vehicle velocity request.
  * Coordinate units are metres/sec and radians/sec.
  * NB: Coordinate frame used is ROS standard [forward, left, up],
  * to simplify integration with arm control systems.
 */
typedef struct cola2_msgs__msg__BodyVelocityReq
{
  /// header.frame_id should be set to the base_link frame of the vehicle.
  std_msgs__msg__Header header;
  cola2_msgs__msg__GoalDescriptor goal;
  geometry_msgs__msg__Twist twist;
  /// Axes of control to disable, in body frame.
  cola2_msgs__msg__Bool6Axis disable_axis;
} cola2_msgs__msg__BodyVelocityReq;

// Struct for a sequence of cola2_msgs__msg__BodyVelocityReq.
typedef struct cola2_msgs__msg__BodyVelocityReq__Sequence
{
  cola2_msgs__msg__BodyVelocityReq * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cola2_msgs__msg__BodyVelocityReq__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // COLA2_MSGS__MSG__DETAIL__BODY_VELOCITY_REQ__STRUCT_H_
