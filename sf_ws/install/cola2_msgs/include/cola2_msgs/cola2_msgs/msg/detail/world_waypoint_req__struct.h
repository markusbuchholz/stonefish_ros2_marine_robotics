// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from cola2_msgs:msg/WorldWaypointReq.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__WORLD_WAYPOINT_REQ__STRUCT_H_
#define COLA2_MSGS__MSG__DETAIL__WORLD_WAYPOINT_REQ__STRUCT_H_

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
// Member 'position'
#include "cola2_msgs/msg/detail/ned__struct.h"
// Member 'orientation'
// Member 'orientation_tolerance'
#include "cola2_msgs/msg/detail/rpy__struct.h"
// Member 'disable_axis'
#include "cola2_msgs/msg/detail/bool6_axis__struct.h"
// Member 'position_tolerance'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/WorldWaypointReq in the package cola2_msgs.
/**
  * World frame (absolute) waypoint request to pilot.
  * A new waypoint request should contain a different goal.id
  * (incremented) from previous requests. This same message should then
  * be re-sent at 5-10 Hz.
 */
typedef struct cola2_msgs__msg__WorldWaypointReq
{
  std_msgs__msg__Header header;
  /// Common waypoint details
  cola2_msgs__msg__GoalDescriptor goal;
  /// If true, maintain Z position relative to altitude, otherwise depth.
  bool altitude_mode;
  cola2_msgs__msg__NED position;
  float altitude;
  cola2_msgs__msg__RPY orientation;
  /// Axes of control to disable, in body frame.
  cola2_msgs__msg__Bool6Axis disable_axis;
  /// Tolerances are in body frame, zero indicates pilot default should be used.
  geometry_msgs__msg__Vector3 position_tolerance;
  cola2_msgs__msg__RPY orientation_tolerance;
} cola2_msgs__msg__WorldWaypointReq;

// Struct for a sequence of cola2_msgs__msg__WorldWaypointReq.
typedef struct cola2_msgs__msg__WorldWaypointReq__Sequence
{
  cola2_msgs__msg__WorldWaypointReq * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cola2_msgs__msg__WorldWaypointReq__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // COLA2_MSGS__MSG__DETAIL__WORLD_WAYPOINT_REQ__STRUCT_H_
