// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from cola2_msgs:srv/Recovery.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__SRV__DETAIL__RECOVERY__STRUCT_H_
#define COLA2_MSGS__SRV__DETAIL__RECOVERY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'requested_action'
#include "cola2_msgs/msg/detail/recovery_action__struct.h"

/// Struct defined in srv/Recovery in the package cola2_msgs.
typedef struct cola2_msgs__srv__Recovery_Request
{
  cola2_msgs__msg__RecoveryAction requested_action;
} cola2_msgs__srv__Recovery_Request;

// Struct for a sequence of cola2_msgs__srv__Recovery_Request.
typedef struct cola2_msgs__srv__Recovery_Request__Sequence
{
  cola2_msgs__srv__Recovery_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cola2_msgs__srv__Recovery_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/Recovery in the package cola2_msgs.
typedef struct cola2_msgs__srv__Recovery_Response
{
  bool attempted;
} cola2_msgs__srv__Recovery_Response;

// Struct for a sequence of cola2_msgs__srv__Recovery_Response.
typedef struct cola2_msgs__srv__Recovery_Response__Sequence
{
  cola2_msgs__srv__Recovery_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cola2_msgs__srv__Recovery_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // COLA2_MSGS__SRV__DETAIL__RECOVERY__STRUCT_H_
