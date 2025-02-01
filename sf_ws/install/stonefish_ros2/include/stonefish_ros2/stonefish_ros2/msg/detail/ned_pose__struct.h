// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from stonefish_ros2:msg/NEDPose.idl
// generated code does not contain a copyright notice

#ifndef STONEFISH_ROS2__MSG__DETAIL__NED_POSE__STRUCT_H_
#define STONEFISH_ROS2__MSG__DETAIL__NED_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/NEDPose in the package stonefish_ros2.
typedef struct stonefish_ros2__msg__NEDPose
{
  double north;
  double east;
  double down;
  double roll;
  double pitch;
  double yaw;
} stonefish_ros2__msg__NEDPose;

// Struct for a sequence of stonefish_ros2__msg__NEDPose.
typedef struct stonefish_ros2__msg__NEDPose__Sequence
{
  stonefish_ros2__msg__NEDPose * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stonefish_ros2__msg__NEDPose__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // STONEFISH_ROS2__MSG__DETAIL__NED_POSE__STRUCT_H_
