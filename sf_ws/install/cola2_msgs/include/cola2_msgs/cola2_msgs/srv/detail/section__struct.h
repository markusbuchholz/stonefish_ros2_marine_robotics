// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from cola2_msgs:srv/Section.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__SRV__DETAIL__SECTION__STRUCT_H_
#define COLA2_MSGS__SRV__DETAIL__SECTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'NED'.
/**
  * Reference frame for both the inital and final positions
 */
enum
{
  cola2_msgs__srv__Section_Request__NED = 0
};

/// Constant 'GLOBAL'.
enum
{
  cola2_msgs__srv__Section_Request__GLOBAL = 1
};

/// Constant 'DEPTH'.
enum
{
  cola2_msgs__srv__Section_Request__DEPTH = 0ull
};

/// Constant 'ALTITUDE'.
enum
{
  cola2_msgs__srv__Section_Request__ALTITUDE = 1ull
};

/// Constant 'BOTH'.
enum
{
  cola2_msgs__srv__Section_Request__BOTH = 2ull
};

/// Struct defined in srv/Section in the package cola2_msgs.
typedef struct cola2_msgs__srv__Section_Request
{
  double initial_x;
  double initial_y;
  double initial_depth;
  /// Final state
  double final_x;
  double final_y;
  double final_depth;
  double final_altitude;
  uint8_t reference;
  /// Heave mode
  uint64_t heave_mode;
  /// Surge velocity
  double surge_velocity;
  /// Tolerance
  double tolerance_xy;
  /// Timeout
  double timeout;
  /// No altitude reactive behavior
  bool no_altitude_goes_up;
} cola2_msgs__srv__Section_Request;

// Struct for a sequence of cola2_msgs__srv__Section_Request.
typedef struct cola2_msgs__srv__Section_Request__Sequence
{
  cola2_msgs__srv__Section_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cola2_msgs__srv__Section_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/Section in the package cola2_msgs.
typedef struct cola2_msgs__srv__Section_Response
{
  bool success;
  rosidl_runtime_c__String message;
} cola2_msgs__srv__Section_Response;

// Struct for a sequence of cola2_msgs__srv__Section_Response.
typedef struct cola2_msgs__srv__Section_Response__Sequence
{
  cola2_msgs__srv__Section_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cola2_msgs__srv__Section_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // COLA2_MSGS__SRV__DETAIL__SECTION__STRUCT_H_
