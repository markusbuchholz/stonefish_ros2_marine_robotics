// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from cola2_msgs:msg/CaptainStateFeedback.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__CAPTAIN_STATE_FEEDBACK__STRUCT_H_
#define COLA2_MSGS__MSG__DETAIL__CAPTAIN_STATE_FEEDBACK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'ACTIVE'.
/**
  * State
 */
enum
{
  cola2_msgs__msg__CaptainStateFeedback__ACTIVE = 0
};

/// Constant 'SUCCESS'.
enum
{
  cola2_msgs__msg__CaptainStateFeedback__SUCCESS = 1
};

/// Constant 'FAILURE'.
enum
{
  cola2_msgs__msg__CaptainStateFeedback__FAILURE = 2
};

/// Constant 'STOPPED'.
enum
{
  cola2_msgs__msg__CaptainStateFeedback__STOPPED = 3
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'name'
#include "rosidl_runtime_c/string.h"
// Member 'keyvalues'
#include "cola2_msgs/msg/detail/key_value__struct.h"

/// Struct defined in msg/CaptainStateFeedback in the package cola2_msgs.
typedef struct cola2_msgs__msg__CaptainStateFeedback
{
  std_msgs__msg__Header header;
  uint8_t state;
  /// An estimate of the remaining time
  double time_to_finish;
  /// Name of the mode
  rosidl_runtime_c__String name;
  /// Always increasing id assigned by the captain
  uint16_t id;
  /// Extra information as a key-value list
  cola2_msgs__msg__KeyValue__Sequence keyvalues;
} cola2_msgs__msg__CaptainStateFeedback;

// Struct for a sequence of cola2_msgs__msg__CaptainStateFeedback.
typedef struct cola2_msgs__msg__CaptainStateFeedback__Sequence
{
  cola2_msgs__msg__CaptainStateFeedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cola2_msgs__msg__CaptainStateFeedback__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // COLA2_MSGS__MSG__DETAIL__CAPTAIN_STATE_FEEDBACK__STRUCT_H_
