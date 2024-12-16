// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from cola2_msgs:msg/RecoveryAction.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__RECOVERY_ACTION__STRUCT_H_
#define COLA2_MSGS__MSG__DETAIL__RECOVERY_ACTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'NONE'.
enum
{
  cola2_msgs__msg__RecoveryAction__NONE = 0
};

/// Constant 'INFORMATIVE'.
enum
{
  cola2_msgs__msg__RecoveryAction__INFORMATIVE = 1
};

/// Constant 'ABORT'.
enum
{
  cola2_msgs__msg__RecoveryAction__ABORT = 2
};

/// Constant 'ABORT_AND_SURFACE'.
enum
{
  cola2_msgs__msg__RecoveryAction__ABORT_AND_SURFACE = 3
};

/// Constant 'EMERGENCY_SURFACE'.
enum
{
  cola2_msgs__msg__RecoveryAction__EMERGENCY_SURFACE = 4
};

/// Constant 'DROP_WEIGHT'.
enum
{
  cola2_msgs__msg__RecoveryAction__DROP_WEIGHT = 5
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'error_string'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/RecoveryAction in the package cola2_msgs.
/**
  * Error level
 */
typedef struct cola2_msgs__msg__RecoveryAction
{
  std_msgs__msg__Header header;
  uint16_t error_level;
  rosidl_runtime_c__String error_string;
} cola2_msgs__msg__RecoveryAction;

// Struct for a sequence of cola2_msgs__msg__RecoveryAction.
typedef struct cola2_msgs__msg__RecoveryAction__Sequence
{
  cola2_msgs__msg__RecoveryAction * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cola2_msgs__msg__RecoveryAction__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // COLA2_MSGS__MSG__DETAIL__RECOVERY_ACTION__STRUCT_H_
