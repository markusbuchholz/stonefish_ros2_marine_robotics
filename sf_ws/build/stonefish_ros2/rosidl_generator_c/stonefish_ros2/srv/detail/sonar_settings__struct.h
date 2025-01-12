// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from stonefish_ros2:srv/SonarSettings.idl
// generated code does not contain a copyright notice

#ifndef STONEFISH_ROS2__SRV__DETAIL__SONAR_SETTINGS__STRUCT_H_
#define STONEFISH_ROS2__SRV__DETAIL__SONAR_SETTINGS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/SonarSettings in the package stonefish_ros2.
typedef struct stonefish_ros2__srv__SonarSettings_Request
{
  double range_min;
  double range_max;
  double gain;
} stonefish_ros2__srv__SonarSettings_Request;

// Struct for a sequence of stonefish_ros2__srv__SonarSettings_Request.
typedef struct stonefish_ros2__srv__SonarSettings_Request__Sequence
{
  stonefish_ros2__srv__SonarSettings_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stonefish_ros2__srv__SonarSettings_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SonarSettings in the package stonefish_ros2.
typedef struct stonefish_ros2__srv__SonarSettings_Response
{
  bool success;
  rosidl_runtime_c__String message;
} stonefish_ros2__srv__SonarSettings_Response;

// Struct for a sequence of stonefish_ros2__srv__SonarSettings_Response.
typedef struct stonefish_ros2__srv__SonarSettings_Response__Sequence
{
  stonefish_ros2__srv__SonarSettings_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stonefish_ros2__srv__SonarSettings_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // STONEFISH_ROS2__SRV__DETAIL__SONAR_SETTINGS__STRUCT_H_
