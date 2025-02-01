// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from cola2_msgs:msg/SafetySupervisorStatus.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__SAFETY_SUPERVISOR_STATUS__STRUCT_H_
#define COLA2_MSGS__MSG__DETAIL__SAFETY_SUPERVISOR_STATUS__STRUCT_H_

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
// Member 'recovery_action'
#include "cola2_msgs/msg/detail/recovery_action__struct.h"

/// Struct defined in msg/SafetySupervisorStatus in the package cola2_msgs.
typedef struct cola2_msgs__msg__SafetySupervisorStatus
{
  std_msgs__msg__Header header;
  uint32_t status_code;
  cola2_msgs__msg__RecoveryAction recovery_action;
} cola2_msgs__msg__SafetySupervisorStatus;

// Struct for a sequence of cola2_msgs__msg__SafetySupervisorStatus.
typedef struct cola2_msgs__msg__SafetySupervisorStatus__Sequence
{
  cola2_msgs__msg__SafetySupervisorStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cola2_msgs__msg__SafetySupervisorStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // COLA2_MSGS__MSG__DETAIL__SAFETY_SUPERVISOR_STATUS__STRUCT_H_
