// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from cola2_msgs:msg/MissionState.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__MISSION_STATE__STRUCT_H_
#define COLA2_MSGS__MSG__DETAIL__MISSION_STATE__STRUCT_H_

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
// Member 'last_active'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in msg/MissionState in the package cola2_msgs.
typedef struct cola2_msgs__msg__MissionState
{
  rosidl_runtime_c__String name;
  int32_t current_step;
  builtin_interfaces__msg__Time last_active;
} cola2_msgs__msg__MissionState;

// Struct for a sequence of cola2_msgs__msg__MissionState.
typedef struct cola2_msgs__msg__MissionState__Sequence
{
  cola2_msgs__msg__MissionState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cola2_msgs__msg__MissionState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // COLA2_MSGS__MSG__DETAIL__MISSION_STATE__STRUCT_H_
