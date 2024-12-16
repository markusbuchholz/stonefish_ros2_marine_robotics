// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from stonefish_ros2:srv/Respawn.idl
// generated code does not contain a copyright notice

#ifndef STONEFISH_ROS2__SRV__DETAIL__RESPAWN__STRUCT_H_
#define STONEFISH_ROS2__SRV__DETAIL__RESPAWN__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'name'
#include "rosidl_runtime_c/string.h"
// Member 'origin'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in srv/Respawn in the package stonefish_ros2.
typedef struct stonefish_ros2__srv__Respawn_Request
{
  rosidl_runtime_c__String name;
  geometry_msgs__msg__Pose origin;
} stonefish_ros2__srv__Respawn_Request;

// Struct for a sequence of stonefish_ros2__srv__Respawn_Request.
typedef struct stonefish_ros2__srv__Respawn_Request__Sequence
{
  stonefish_ros2__srv__Respawn_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stonefish_ros2__srv__Respawn_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/Respawn in the package stonefish_ros2.
typedef struct stonefish_ros2__srv__Respawn_Response
{
  bool success;
  rosidl_runtime_c__String message;
} stonefish_ros2__srv__Respawn_Response;

// Struct for a sequence of stonefish_ros2__srv__Respawn_Response.
typedef struct stonefish_ros2__srv__Respawn_Response__Sequence
{
  stonefish_ros2__srv__Respawn_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stonefish_ros2__srv__Respawn_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // STONEFISH_ROS2__SRV__DETAIL__RESPAWN__STRUCT_H_
