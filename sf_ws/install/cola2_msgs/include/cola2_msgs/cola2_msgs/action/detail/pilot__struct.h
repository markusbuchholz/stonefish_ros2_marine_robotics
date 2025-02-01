// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from cola2_msgs:action/Pilot.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__ACTION__DETAIL__PILOT__STRUCT_H_
#define COLA2_MSGS__ACTION__DETAIL__PILOT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'DEPTH'.
enum
{
  cola2_msgs__action__Pilot_Goal__DEPTH = 0ull
};

/// Constant 'ALTITUDE'.
enum
{
  cola2_msgs__action__Pilot_Goal__ALTITUDE = 1ull
};

/// Constant 'BOTH'.
enum
{
  cola2_msgs__action__Pilot_Goal__BOTH = 2ull
};

/// Constant 'SECTION'.
enum
{
  cola2_msgs__action__Pilot_Goal__SECTION = 0ull
};

/// Constant 'ANCHOR'.
enum
{
  cola2_msgs__action__Pilot_Goal__ANCHOR = 1ull
};

/// Constant 'HOLONOMIC_KEEP_POSITION'.
enum
{
  cola2_msgs__action__Pilot_Goal__HOLONOMIC_KEEP_POSITION = 2ull
};

// Include directives for member types
// Member 'goal'
#include "cola2_msgs/msg/detail/goal_descriptor__struct.h"

/// Struct defined in action/Pilot in the package cola2_msgs.
typedef struct cola2_msgs__action__Pilot_Goal
{
  /// Initial state
  double initial_latitude;
  double initial_longitude;
  double initial_depth;
  /// Final state
  double final_latitude;
  double final_longitude;
  double final_depth;
  double final_yaw;
  double final_altitude;
  /// Heave mode
  uint64_t heave_mode;
  /// Surge velocity
  double surge_velocity;
  /// Tolerance
  double tolerance_xy;
  /// Timeout
  double timeout;
  /// Controller type
  uint64_t controller_type;
  /// Requester and priority
  cola2_msgs__msg__GoalDescriptor goal;
} cola2_msgs__action__Pilot_Goal;

// Struct for a sequence of cola2_msgs__action__Pilot_Goal.
typedef struct cola2_msgs__action__Pilot_Goal__Sequence
{
  cola2_msgs__action__Pilot_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cola2_msgs__action__Pilot_Goal__Sequence;


// Constants defined in the message

/// Constant 'SUCCESS'.
enum
{
  cola2_msgs__action__Pilot_Result__SUCCESS = 0ull
};

/// Constant 'TIMEOUT'.
enum
{
  cola2_msgs__action__Pilot_Result__TIMEOUT = 1ull
};

/// Constant 'FAILURE'.
enum
{
  cola2_msgs__action__Pilot_Result__FAILURE = 2ull
};

/// Struct defined in action/Pilot in the package cola2_msgs.
typedef struct cola2_msgs__action__Pilot_Result
{
  /// State
  uint64_t state;
} cola2_msgs__action__Pilot_Result;

// Struct for a sequence of cola2_msgs__action__Pilot_Result.
typedef struct cola2_msgs__action__Pilot_Result__Sequence
{
  cola2_msgs__action__Pilot_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cola2_msgs__action__Pilot_Result__Sequence;


// Constants defined in the message

/// Struct defined in action/Pilot in the package cola2_msgs.
typedef struct cola2_msgs__action__Pilot_Feedback
{
  /// Errors
  double distance_to_end;
  double cross_track_error;
} cola2_msgs__action__Pilot_Feedback;

// Struct for a sequence of cola2_msgs__action__Pilot_Feedback.
typedef struct cola2_msgs__action__Pilot_Feedback__Sequence
{
  cola2_msgs__action__Pilot_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cola2_msgs__action__Pilot_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "cola2_msgs/action/detail/pilot__struct.h"

/// Struct defined in action/Pilot in the package cola2_msgs.
typedef struct cola2_msgs__action__Pilot_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  cola2_msgs__action__Pilot_Goal goal;
} cola2_msgs__action__Pilot_SendGoal_Request;

// Struct for a sequence of cola2_msgs__action__Pilot_SendGoal_Request.
typedef struct cola2_msgs__action__Pilot_SendGoal_Request__Sequence
{
  cola2_msgs__action__Pilot_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cola2_msgs__action__Pilot_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/Pilot in the package cola2_msgs.
typedef struct cola2_msgs__action__Pilot_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} cola2_msgs__action__Pilot_SendGoal_Response;

// Struct for a sequence of cola2_msgs__action__Pilot_SendGoal_Response.
typedef struct cola2_msgs__action__Pilot_SendGoal_Response__Sequence
{
  cola2_msgs__action__Pilot_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cola2_msgs__action__Pilot_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/Pilot in the package cola2_msgs.
typedef struct cola2_msgs__action__Pilot_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} cola2_msgs__action__Pilot_GetResult_Request;

// Struct for a sequence of cola2_msgs__action__Pilot_GetResult_Request.
typedef struct cola2_msgs__action__Pilot_GetResult_Request__Sequence
{
  cola2_msgs__action__Pilot_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cola2_msgs__action__Pilot_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "cola2_msgs/action/detail/pilot__struct.h"

/// Struct defined in action/Pilot in the package cola2_msgs.
typedef struct cola2_msgs__action__Pilot_GetResult_Response
{
  int8_t status;
  cola2_msgs__action__Pilot_Result result;
} cola2_msgs__action__Pilot_GetResult_Response;

// Struct for a sequence of cola2_msgs__action__Pilot_GetResult_Response.
typedef struct cola2_msgs__action__Pilot_GetResult_Response__Sequence
{
  cola2_msgs__action__Pilot_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cola2_msgs__action__Pilot_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "cola2_msgs/action/detail/pilot__struct.h"

/// Struct defined in action/Pilot in the package cola2_msgs.
typedef struct cola2_msgs__action__Pilot_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  cola2_msgs__action__Pilot_Feedback feedback;
} cola2_msgs__action__Pilot_FeedbackMessage;

// Struct for a sequence of cola2_msgs__action__Pilot_FeedbackMessage.
typedef struct cola2_msgs__action__Pilot_FeedbackMessage__Sequence
{
  cola2_msgs__action__Pilot_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cola2_msgs__action__Pilot_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // COLA2_MSGS__ACTION__DETAIL__PILOT__STRUCT_H_
