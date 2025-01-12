// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from cola2_msgs:msg/CaptainStatus.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__CAPTAIN_STATUS__STRUCT_H_
#define COLA2_MSGS__MSG__DETAIL__CAPTAIN_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'IDLE'.
/**
  * State
 */
enum
{
  cola2_msgs__msg__CaptainStatus__IDLE = 0
};

/// Constant 'GOTO'.
enum
{
  cola2_msgs__msg__CaptainStatus__GOTO = 1
};

/// Constant 'MISSION'.
enum
{
  cola2_msgs__msg__CaptainStatus__MISSION = 2
};

/// Constant 'KEEPPOSITION'.
enum
{
  cola2_msgs__msg__CaptainStatus__KEEPPOSITION = 3
};

/// Constant 'SAFETYKEEPPOSITION'.
enum
{
  cola2_msgs__msg__CaptainStatus__SAFETYKEEPPOSITION = 4
};

/// Constant 'EXTERNALMISSION'.
enum
{
  cola2_msgs__msg__CaptainStatus__EXTERNALMISSION = 5
};

/// Constant 'SECTION'.
enum
{
  cola2_msgs__msg__CaptainStatus__SECTION = 6
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'message'
#include "rosidl_runtime_c/string.h"
// Member 'loaded_missions'
#include "cola2_msgs/msg/detail/mission_state__struct.h"

/// Struct defined in msg/CaptainStatus in the package cola2_msgs.
typedef struct cola2_msgs__msg__CaptainStatus
{
  std_msgs__msg__Header header;
  uint8_t state;
  /// Info message
  rosidl_runtime_c__String message;
  /// List of missions
  cola2_msgs__msg__MissionState__Sequence loaded_missions;
} cola2_msgs__msg__CaptainStatus;

// Struct for a sequence of cola2_msgs__msg__CaptainStatus.
typedef struct cola2_msgs__msg__CaptainStatus__Sequence
{
  cola2_msgs__msg__CaptainStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cola2_msgs__msg__CaptainStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // COLA2_MSGS__MSG__DETAIL__CAPTAIN_STATUS__STRUCT_H_
