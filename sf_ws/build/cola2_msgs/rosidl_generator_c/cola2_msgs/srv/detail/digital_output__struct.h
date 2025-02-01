// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from cola2_msgs:srv/DigitalOutput.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__SRV__DETAIL__DIGITAL_OUTPUT__STRUCT_H_
#define COLA2_MSGS__SRV__DETAIL__DIGITAL_OUTPUT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/DigitalOutput in the package cola2_msgs.
typedef struct cola2_msgs__srv__DigitalOutput_Request
{
  int32_t digital_output;
  int32_t value;
} cola2_msgs__srv__DigitalOutput_Request;

// Struct for a sequence of cola2_msgs__srv__DigitalOutput_Request.
typedef struct cola2_msgs__srv__DigitalOutput_Request__Sequence
{
  cola2_msgs__srv__DigitalOutput_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cola2_msgs__srv__DigitalOutput_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/DigitalOutput in the package cola2_msgs.
typedef struct cola2_msgs__srv__DigitalOutput_Response
{
  bool success;
  rosidl_runtime_c__String message;
} cola2_msgs__srv__DigitalOutput_Response;

// Struct for a sequence of cola2_msgs__srv__DigitalOutput_Response.
typedef struct cola2_msgs__srv__DigitalOutput_Response__Sequence
{
  cola2_msgs__srv__DigitalOutput_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cola2_msgs__srv__DigitalOutput_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // COLA2_MSGS__SRV__DETAIL__DIGITAL_OUTPUT__STRUCT_H_
