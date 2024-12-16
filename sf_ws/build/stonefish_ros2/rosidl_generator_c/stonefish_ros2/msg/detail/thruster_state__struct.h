// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from stonefish_ros2:msg/ThrusterState.idl
// generated code does not contain a copyright notice

#ifndef STONEFISH_ROS2__MSG__DETAIL__THRUSTER_STATE__STRUCT_H_
#define STONEFISH_ROS2__MSG__DETAIL__THRUSTER_STATE__STRUCT_H_

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
// Member 'setpoint'
// Member 'rpm'
// Member 'thrust'
// Member 'torque'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/ThrusterState in the package stonefish_ros2.
typedef struct stonefish_ros2__msg__ThrusterState
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__double__Sequence setpoint;
  rosidl_runtime_c__double__Sequence rpm;
  rosidl_runtime_c__double__Sequence thrust;
  rosidl_runtime_c__double__Sequence torque;
} stonefish_ros2__msg__ThrusterState;

// Struct for a sequence of stonefish_ros2__msg__ThrusterState.
typedef struct stonefish_ros2__msg__ThrusterState__Sequence
{
  stonefish_ros2__msg__ThrusterState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stonefish_ros2__msg__ThrusterState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // STONEFISH_ROS2__MSG__DETAIL__THRUSTER_STATE__STRUCT_H_
