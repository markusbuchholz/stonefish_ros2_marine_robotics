// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from stonefish_ros2:msg/DebugPhysics.idl
// generated code does not contain a copyright notice

#ifndef STONEFISH_ROS2__MSG__DETAIL__DEBUG_PHYSICS__STRUCT_H_
#define STONEFISH_ROS2__MSG__DETAIL__DEBUG_PHYSICS__STRUCT_H_

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
// Member 'cog'
// Member 'cob'
// Member 'inertia'
// Member 'damping_coeff'
// Member 'skin_friction_coeff'
#include "geometry_msgs/msg/detail/vector3__struct.h"
// Member 'velocity'
#include "geometry_msgs/msg/detail/twist__struct.h"
// Member 'buoyancy'
// Member 'damping'
// Member 'skin_friction'
#include "geometry_msgs/msg/detail/wrench__struct.h"

/// Struct defined in msg/DebugPhysics in the package stonefish_ros2.
typedef struct stonefish_ros2__msg__DebugPhysics
{
  std_msgs__msg__Header header;
  double mass;
  double volume;
  double surface;
  /// In local frame (origin)
  geometry_msgs__msg__Vector3 cog;
  geometry_msgs__msg__Vector3 cob;
  geometry_msgs__msg__Vector3 inertia;
  geometry_msgs__msg__Vector3 damping_coeff;
  geometry_msgs__msg__Vector3 skin_friction_coeff;
  geometry_msgs__msg__Twist velocity;
  geometry_msgs__msg__Wrench buoyancy;
  geometry_msgs__msg__Wrench damping;
  geometry_msgs__msg__Wrench skin_friction;
  double wetted_surface;
  double submerged_volume;
} stonefish_ros2__msg__DebugPhysics;

// Struct for a sequence of stonefish_ros2__msg__DebugPhysics.
typedef struct stonefish_ros2__msg__DebugPhysics__Sequence
{
  stonefish_ros2__msg__DebugPhysics * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stonefish_ros2__msg__DebugPhysics__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // STONEFISH_ROS2__MSG__DETAIL__DEBUG_PHYSICS__STRUCT_H_
