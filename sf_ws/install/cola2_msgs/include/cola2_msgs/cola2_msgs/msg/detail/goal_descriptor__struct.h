// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from cola2_msgs:msg/GoalDescriptor.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__GOAL_DESCRIPTOR__STRUCT_H_
#define COLA2_MSGS__MSG__DETAIL__GOAL_DESCRIPTOR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'PRIORITY_TELEOPERATION_LOW'.
/**
  * PRIORITY DEFINITIONS
 */
enum
{
  cola2_msgs__msg__GoalDescriptor__PRIORITY_TELEOPERATION_LOW = 0ul
};

/// Constant 'PRIORITY_SAFETY_LOW'.
enum
{
  cola2_msgs__msg__GoalDescriptor__PRIORITY_SAFETY_LOW = 5ul
};

/// Constant 'PRIORITY_NORMAL'.
enum
{
  cola2_msgs__msg__GoalDescriptor__PRIORITY_NORMAL = 10ul
};

/// Constant 'PRIORITY_SAFETY'.
enum
{
  cola2_msgs__msg__GoalDescriptor__PRIORITY_SAFETY = 30ul
};

/// Constant 'PRIORITY_TELEOPERATION'.
enum
{
  cola2_msgs__msg__GoalDescriptor__PRIORITY_TELEOPERATION = 40ul
};

/// Constant 'PRIORITY_SAFETY_HIGH'.
enum
{
  cola2_msgs__msg__GoalDescriptor__PRIORITY_SAFETY_HIGH = 50ul
};

/// Constant 'PRIORITY_TELEOPERATION_HIGH'.
enum
{
  cola2_msgs__msg__GoalDescriptor__PRIORITY_TELEOPERATION_HIGH = 60ul
};

// Include directives for member types
// Member 'requester'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/GoalDescriptor in the package cola2_msgs.
/**
  * Contains values common to all waypoint request and status messages.
  * Waypoint goals with higher priority trump lower priority goals. For
  * goals of the same priority, the pilot should follow the latest
  * received.
 */
typedef struct cola2_msgs__msg__GoalDescriptor
{
  /// Name of the requesting node, used with id to identify the request.
  rosidl_runtime_c__String requester;
  uint32_t priority;
} cola2_msgs__msg__GoalDescriptor;

// Struct for a sequence of cola2_msgs__msg__GoalDescriptor.
typedef struct cola2_msgs__msg__GoalDescriptor__Sequence
{
  cola2_msgs__msg__GoalDescriptor * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cola2_msgs__msg__GoalDescriptor__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // COLA2_MSGS__MSG__DETAIL__GOAL_DESCRIPTOR__STRUCT_H_
