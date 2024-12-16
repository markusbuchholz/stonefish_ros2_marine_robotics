// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from cola2_msgs:srv/Mission.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__SRV__DETAIL__MISSION__STRUCT_H_
#define COLA2_MSGS__SRV__DETAIL__MISSION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'mission'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/Mission in the package cola2_msgs.
typedef struct cola2_msgs__srv__Mission_Request
{
  rosidl_runtime_c__String mission;
} cola2_msgs__srv__Mission_Request;

// Struct for a sequence of cola2_msgs__srv__Mission_Request.
typedef struct cola2_msgs__srv__Mission_Request__Sequence
{
  cola2_msgs__srv__Mission_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cola2_msgs__srv__Mission_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/Mission in the package cola2_msgs.
typedef struct cola2_msgs__srv__Mission_Response
{
  bool success;
  rosidl_runtime_c__String message;
} cola2_msgs__srv__Mission_Response;

// Struct for a sequence of cola2_msgs__srv__Mission_Response.
typedef struct cola2_msgs__srv__Mission_Response__Sequence
{
  cola2_msgs__srv__Mission_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cola2_msgs__srv__Mission_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // COLA2_MSGS__SRV__DETAIL__MISSION__STRUCT_H_
