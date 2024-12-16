// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from cola2_msgs:action/Pilot.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "cola2_msgs/action/detail/pilot__rosidl_typesupport_introspection_c.h"
#include "cola2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "cola2_msgs/action/detail/pilot__functions.h"
#include "cola2_msgs/action/detail/pilot__struct.h"


// Include directives for member types
// Member `goal`
#include "cola2_msgs/msg/goal_descriptor.h"
// Member `goal`
#include "cola2_msgs/msg/detail/goal_descriptor__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void cola2_msgs__action__Pilot_Goal__rosidl_typesupport_introspection_c__Pilot_Goal_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  cola2_msgs__action__Pilot_Goal__init(message_memory);
}

void cola2_msgs__action__Pilot_Goal__rosidl_typesupport_introspection_c__Pilot_Goal_fini_function(void * message_memory)
{
  cola2_msgs__action__Pilot_Goal__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember cola2_msgs__action__Pilot_Goal__rosidl_typesupport_introspection_c__Pilot_Goal_message_member_array[14] = {
  {
    "initial_latitude",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__action__Pilot_Goal, initial_latitude),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "initial_longitude",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__action__Pilot_Goal, initial_longitude),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "initial_depth",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__action__Pilot_Goal, initial_depth),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "final_latitude",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__action__Pilot_Goal, final_latitude),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "final_longitude",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__action__Pilot_Goal, final_longitude),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "final_depth",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__action__Pilot_Goal, final_depth),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "final_yaw",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__action__Pilot_Goal, final_yaw),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "final_altitude",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__action__Pilot_Goal, final_altitude),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "heave_mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__action__Pilot_Goal, heave_mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "surge_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__action__Pilot_Goal, surge_velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "tolerance_xy",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__action__Pilot_Goal, tolerance_xy),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "timeout",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__action__Pilot_Goal, timeout),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "controller_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__action__Pilot_Goal, controller_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "goal",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__action__Pilot_Goal, goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers cola2_msgs__action__Pilot_Goal__rosidl_typesupport_introspection_c__Pilot_Goal_message_members = {
  "cola2_msgs__action",  // message namespace
  "Pilot_Goal",  // message name
  14,  // number of fields
  sizeof(cola2_msgs__action__Pilot_Goal),
  cola2_msgs__action__Pilot_Goal__rosidl_typesupport_introspection_c__Pilot_Goal_message_member_array,  // message members
  cola2_msgs__action__Pilot_Goal__rosidl_typesupport_introspection_c__Pilot_Goal_init_function,  // function to initialize message memory (memory has to be allocated)
  cola2_msgs__action__Pilot_Goal__rosidl_typesupport_introspection_c__Pilot_Goal_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t cola2_msgs__action__Pilot_Goal__rosidl_typesupport_introspection_c__Pilot_Goal_message_type_support_handle = {
  0,
  &cola2_msgs__action__Pilot_Goal__rosidl_typesupport_introspection_c__Pilot_Goal_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_cola2_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cola2_msgs, action, Pilot_Goal)() {
  cola2_msgs__action__Pilot_Goal__rosidl_typesupport_introspection_c__Pilot_Goal_message_member_array[13].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cola2_msgs, msg, GoalDescriptor)();
  if (!cola2_msgs__action__Pilot_Goal__rosidl_typesupport_introspection_c__Pilot_Goal_message_type_support_handle.typesupport_identifier) {
    cola2_msgs__action__Pilot_Goal__rosidl_typesupport_introspection_c__Pilot_Goal_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &cola2_msgs__action__Pilot_Goal__rosidl_typesupport_introspection_c__Pilot_Goal_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "cola2_msgs/action/detail/pilot__rosidl_typesupport_introspection_c.h"
// already included above
// #include "cola2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "cola2_msgs/action/detail/pilot__functions.h"
// already included above
// #include "cola2_msgs/action/detail/pilot__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void cola2_msgs__action__Pilot_Result__rosidl_typesupport_introspection_c__Pilot_Result_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  cola2_msgs__action__Pilot_Result__init(message_memory);
}

void cola2_msgs__action__Pilot_Result__rosidl_typesupport_introspection_c__Pilot_Result_fini_function(void * message_memory)
{
  cola2_msgs__action__Pilot_Result__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember cola2_msgs__action__Pilot_Result__rosidl_typesupport_introspection_c__Pilot_Result_message_member_array[1] = {
  {
    "state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__action__Pilot_Result, state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers cola2_msgs__action__Pilot_Result__rosidl_typesupport_introspection_c__Pilot_Result_message_members = {
  "cola2_msgs__action",  // message namespace
  "Pilot_Result",  // message name
  1,  // number of fields
  sizeof(cola2_msgs__action__Pilot_Result),
  cola2_msgs__action__Pilot_Result__rosidl_typesupport_introspection_c__Pilot_Result_message_member_array,  // message members
  cola2_msgs__action__Pilot_Result__rosidl_typesupport_introspection_c__Pilot_Result_init_function,  // function to initialize message memory (memory has to be allocated)
  cola2_msgs__action__Pilot_Result__rosidl_typesupport_introspection_c__Pilot_Result_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t cola2_msgs__action__Pilot_Result__rosidl_typesupport_introspection_c__Pilot_Result_message_type_support_handle = {
  0,
  &cola2_msgs__action__Pilot_Result__rosidl_typesupport_introspection_c__Pilot_Result_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_cola2_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cola2_msgs, action, Pilot_Result)() {
  if (!cola2_msgs__action__Pilot_Result__rosidl_typesupport_introspection_c__Pilot_Result_message_type_support_handle.typesupport_identifier) {
    cola2_msgs__action__Pilot_Result__rosidl_typesupport_introspection_c__Pilot_Result_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &cola2_msgs__action__Pilot_Result__rosidl_typesupport_introspection_c__Pilot_Result_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "cola2_msgs/action/detail/pilot__rosidl_typesupport_introspection_c.h"
// already included above
// #include "cola2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "cola2_msgs/action/detail/pilot__functions.h"
// already included above
// #include "cola2_msgs/action/detail/pilot__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void cola2_msgs__action__Pilot_Feedback__rosidl_typesupport_introspection_c__Pilot_Feedback_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  cola2_msgs__action__Pilot_Feedback__init(message_memory);
}

void cola2_msgs__action__Pilot_Feedback__rosidl_typesupport_introspection_c__Pilot_Feedback_fini_function(void * message_memory)
{
  cola2_msgs__action__Pilot_Feedback__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember cola2_msgs__action__Pilot_Feedback__rosidl_typesupport_introspection_c__Pilot_Feedback_message_member_array[2] = {
  {
    "distance_to_end",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__action__Pilot_Feedback, distance_to_end),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cross_track_error",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__action__Pilot_Feedback, cross_track_error),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers cola2_msgs__action__Pilot_Feedback__rosidl_typesupport_introspection_c__Pilot_Feedback_message_members = {
  "cola2_msgs__action",  // message namespace
  "Pilot_Feedback",  // message name
  2,  // number of fields
  sizeof(cola2_msgs__action__Pilot_Feedback),
  cola2_msgs__action__Pilot_Feedback__rosidl_typesupport_introspection_c__Pilot_Feedback_message_member_array,  // message members
  cola2_msgs__action__Pilot_Feedback__rosidl_typesupport_introspection_c__Pilot_Feedback_init_function,  // function to initialize message memory (memory has to be allocated)
  cola2_msgs__action__Pilot_Feedback__rosidl_typesupport_introspection_c__Pilot_Feedback_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t cola2_msgs__action__Pilot_Feedback__rosidl_typesupport_introspection_c__Pilot_Feedback_message_type_support_handle = {
  0,
  &cola2_msgs__action__Pilot_Feedback__rosidl_typesupport_introspection_c__Pilot_Feedback_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_cola2_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cola2_msgs, action, Pilot_Feedback)() {
  if (!cola2_msgs__action__Pilot_Feedback__rosidl_typesupport_introspection_c__Pilot_Feedback_message_type_support_handle.typesupport_identifier) {
    cola2_msgs__action__Pilot_Feedback__rosidl_typesupport_introspection_c__Pilot_Feedback_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &cola2_msgs__action__Pilot_Feedback__rosidl_typesupport_introspection_c__Pilot_Feedback_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "cola2_msgs/action/detail/pilot__rosidl_typesupport_introspection_c.h"
// already included above
// #include "cola2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "cola2_msgs/action/detail/pilot__functions.h"
// already included above
// #include "cola2_msgs/action/detail/pilot__struct.h"


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `goal`
#include "cola2_msgs/action/pilot.h"
// Member `goal`
// already included above
// #include "cola2_msgs/action/detail/pilot__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void cola2_msgs__action__Pilot_SendGoal_Request__rosidl_typesupport_introspection_c__Pilot_SendGoal_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  cola2_msgs__action__Pilot_SendGoal_Request__init(message_memory);
}

void cola2_msgs__action__Pilot_SendGoal_Request__rosidl_typesupport_introspection_c__Pilot_SendGoal_Request_fini_function(void * message_memory)
{
  cola2_msgs__action__Pilot_SendGoal_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember cola2_msgs__action__Pilot_SendGoal_Request__rosidl_typesupport_introspection_c__Pilot_SendGoal_Request_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__action__Pilot_SendGoal_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "goal",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__action__Pilot_SendGoal_Request, goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers cola2_msgs__action__Pilot_SendGoal_Request__rosidl_typesupport_introspection_c__Pilot_SendGoal_Request_message_members = {
  "cola2_msgs__action",  // message namespace
  "Pilot_SendGoal_Request",  // message name
  2,  // number of fields
  sizeof(cola2_msgs__action__Pilot_SendGoal_Request),
  cola2_msgs__action__Pilot_SendGoal_Request__rosidl_typesupport_introspection_c__Pilot_SendGoal_Request_message_member_array,  // message members
  cola2_msgs__action__Pilot_SendGoal_Request__rosidl_typesupport_introspection_c__Pilot_SendGoal_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  cola2_msgs__action__Pilot_SendGoal_Request__rosidl_typesupport_introspection_c__Pilot_SendGoal_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t cola2_msgs__action__Pilot_SendGoal_Request__rosidl_typesupport_introspection_c__Pilot_SendGoal_Request_message_type_support_handle = {
  0,
  &cola2_msgs__action__Pilot_SendGoal_Request__rosidl_typesupport_introspection_c__Pilot_SendGoal_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_cola2_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cola2_msgs, action, Pilot_SendGoal_Request)() {
  cola2_msgs__action__Pilot_SendGoal_Request__rosidl_typesupport_introspection_c__Pilot_SendGoal_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  cola2_msgs__action__Pilot_SendGoal_Request__rosidl_typesupport_introspection_c__Pilot_SendGoal_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cola2_msgs, action, Pilot_Goal)();
  if (!cola2_msgs__action__Pilot_SendGoal_Request__rosidl_typesupport_introspection_c__Pilot_SendGoal_Request_message_type_support_handle.typesupport_identifier) {
    cola2_msgs__action__Pilot_SendGoal_Request__rosidl_typesupport_introspection_c__Pilot_SendGoal_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &cola2_msgs__action__Pilot_SendGoal_Request__rosidl_typesupport_introspection_c__Pilot_SendGoal_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "cola2_msgs/action/detail/pilot__rosidl_typesupport_introspection_c.h"
// already included above
// #include "cola2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "cola2_msgs/action/detail/pilot__functions.h"
// already included above
// #include "cola2_msgs/action/detail/pilot__struct.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/time.h"
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void cola2_msgs__action__Pilot_SendGoal_Response__rosidl_typesupport_introspection_c__Pilot_SendGoal_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  cola2_msgs__action__Pilot_SendGoal_Response__init(message_memory);
}

void cola2_msgs__action__Pilot_SendGoal_Response__rosidl_typesupport_introspection_c__Pilot_SendGoal_Response_fini_function(void * message_memory)
{
  cola2_msgs__action__Pilot_SendGoal_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember cola2_msgs__action__Pilot_SendGoal_Response__rosidl_typesupport_introspection_c__Pilot_SendGoal_Response_message_member_array[2] = {
  {
    "accepted",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__action__Pilot_SendGoal_Response, accepted),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__action__Pilot_SendGoal_Response, stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers cola2_msgs__action__Pilot_SendGoal_Response__rosidl_typesupport_introspection_c__Pilot_SendGoal_Response_message_members = {
  "cola2_msgs__action",  // message namespace
  "Pilot_SendGoal_Response",  // message name
  2,  // number of fields
  sizeof(cola2_msgs__action__Pilot_SendGoal_Response),
  cola2_msgs__action__Pilot_SendGoal_Response__rosidl_typesupport_introspection_c__Pilot_SendGoal_Response_message_member_array,  // message members
  cola2_msgs__action__Pilot_SendGoal_Response__rosidl_typesupport_introspection_c__Pilot_SendGoal_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  cola2_msgs__action__Pilot_SendGoal_Response__rosidl_typesupport_introspection_c__Pilot_SendGoal_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t cola2_msgs__action__Pilot_SendGoal_Response__rosidl_typesupport_introspection_c__Pilot_SendGoal_Response_message_type_support_handle = {
  0,
  &cola2_msgs__action__Pilot_SendGoal_Response__rosidl_typesupport_introspection_c__Pilot_SendGoal_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_cola2_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cola2_msgs, action, Pilot_SendGoal_Response)() {
  cola2_msgs__action__Pilot_SendGoal_Response__rosidl_typesupport_introspection_c__Pilot_SendGoal_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!cola2_msgs__action__Pilot_SendGoal_Response__rosidl_typesupport_introspection_c__Pilot_SendGoal_Response_message_type_support_handle.typesupport_identifier) {
    cola2_msgs__action__Pilot_SendGoal_Response__rosidl_typesupport_introspection_c__Pilot_SendGoal_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &cola2_msgs__action__Pilot_SendGoal_Response__rosidl_typesupport_introspection_c__Pilot_SendGoal_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "cola2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "cola2_msgs/action/detail/pilot__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers cola2_msgs__action__detail__pilot__rosidl_typesupport_introspection_c__Pilot_SendGoal_service_members = {
  "cola2_msgs__action",  // service namespace
  "Pilot_SendGoal",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // cola2_msgs__action__detail__pilot__rosidl_typesupport_introspection_c__Pilot_SendGoal_Request_message_type_support_handle,
  NULL  // response message
  // cola2_msgs__action__detail__pilot__rosidl_typesupport_introspection_c__Pilot_SendGoal_Response_message_type_support_handle
};

static rosidl_service_type_support_t cola2_msgs__action__detail__pilot__rosidl_typesupport_introspection_c__Pilot_SendGoal_service_type_support_handle = {
  0,
  &cola2_msgs__action__detail__pilot__rosidl_typesupport_introspection_c__Pilot_SendGoal_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cola2_msgs, action, Pilot_SendGoal_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cola2_msgs, action, Pilot_SendGoal_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_cola2_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cola2_msgs, action, Pilot_SendGoal)() {
  if (!cola2_msgs__action__detail__pilot__rosidl_typesupport_introspection_c__Pilot_SendGoal_service_type_support_handle.typesupport_identifier) {
    cola2_msgs__action__detail__pilot__rosidl_typesupport_introspection_c__Pilot_SendGoal_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)cola2_msgs__action__detail__pilot__rosidl_typesupport_introspection_c__Pilot_SendGoal_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cola2_msgs, action, Pilot_SendGoal_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cola2_msgs, action, Pilot_SendGoal_Response)()->data;
  }

  return &cola2_msgs__action__detail__pilot__rosidl_typesupport_introspection_c__Pilot_SendGoal_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "cola2_msgs/action/detail/pilot__rosidl_typesupport_introspection_c.h"
// already included above
// #include "cola2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "cola2_msgs/action/detail/pilot__functions.h"
// already included above
// #include "cola2_msgs/action/detail/pilot__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void cola2_msgs__action__Pilot_GetResult_Request__rosidl_typesupport_introspection_c__Pilot_GetResult_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  cola2_msgs__action__Pilot_GetResult_Request__init(message_memory);
}

void cola2_msgs__action__Pilot_GetResult_Request__rosidl_typesupport_introspection_c__Pilot_GetResult_Request_fini_function(void * message_memory)
{
  cola2_msgs__action__Pilot_GetResult_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember cola2_msgs__action__Pilot_GetResult_Request__rosidl_typesupport_introspection_c__Pilot_GetResult_Request_message_member_array[1] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__action__Pilot_GetResult_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers cola2_msgs__action__Pilot_GetResult_Request__rosidl_typesupport_introspection_c__Pilot_GetResult_Request_message_members = {
  "cola2_msgs__action",  // message namespace
  "Pilot_GetResult_Request",  // message name
  1,  // number of fields
  sizeof(cola2_msgs__action__Pilot_GetResult_Request),
  cola2_msgs__action__Pilot_GetResult_Request__rosidl_typesupport_introspection_c__Pilot_GetResult_Request_message_member_array,  // message members
  cola2_msgs__action__Pilot_GetResult_Request__rosidl_typesupport_introspection_c__Pilot_GetResult_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  cola2_msgs__action__Pilot_GetResult_Request__rosidl_typesupport_introspection_c__Pilot_GetResult_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t cola2_msgs__action__Pilot_GetResult_Request__rosidl_typesupport_introspection_c__Pilot_GetResult_Request_message_type_support_handle = {
  0,
  &cola2_msgs__action__Pilot_GetResult_Request__rosidl_typesupport_introspection_c__Pilot_GetResult_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_cola2_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cola2_msgs, action, Pilot_GetResult_Request)() {
  cola2_msgs__action__Pilot_GetResult_Request__rosidl_typesupport_introspection_c__Pilot_GetResult_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  if (!cola2_msgs__action__Pilot_GetResult_Request__rosidl_typesupport_introspection_c__Pilot_GetResult_Request_message_type_support_handle.typesupport_identifier) {
    cola2_msgs__action__Pilot_GetResult_Request__rosidl_typesupport_introspection_c__Pilot_GetResult_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &cola2_msgs__action__Pilot_GetResult_Request__rosidl_typesupport_introspection_c__Pilot_GetResult_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "cola2_msgs/action/detail/pilot__rosidl_typesupport_introspection_c.h"
// already included above
// #include "cola2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "cola2_msgs/action/detail/pilot__functions.h"
// already included above
// #include "cola2_msgs/action/detail/pilot__struct.h"


// Include directives for member types
// Member `result`
// already included above
// #include "cola2_msgs/action/pilot.h"
// Member `result`
// already included above
// #include "cola2_msgs/action/detail/pilot__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void cola2_msgs__action__Pilot_GetResult_Response__rosidl_typesupport_introspection_c__Pilot_GetResult_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  cola2_msgs__action__Pilot_GetResult_Response__init(message_memory);
}

void cola2_msgs__action__Pilot_GetResult_Response__rosidl_typesupport_introspection_c__Pilot_GetResult_Response_fini_function(void * message_memory)
{
  cola2_msgs__action__Pilot_GetResult_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember cola2_msgs__action__Pilot_GetResult_Response__rosidl_typesupport_introspection_c__Pilot_GetResult_Response_message_member_array[2] = {
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__action__Pilot_GetResult_Response, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "result",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__action__Pilot_GetResult_Response, result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers cola2_msgs__action__Pilot_GetResult_Response__rosidl_typesupport_introspection_c__Pilot_GetResult_Response_message_members = {
  "cola2_msgs__action",  // message namespace
  "Pilot_GetResult_Response",  // message name
  2,  // number of fields
  sizeof(cola2_msgs__action__Pilot_GetResult_Response),
  cola2_msgs__action__Pilot_GetResult_Response__rosidl_typesupport_introspection_c__Pilot_GetResult_Response_message_member_array,  // message members
  cola2_msgs__action__Pilot_GetResult_Response__rosidl_typesupport_introspection_c__Pilot_GetResult_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  cola2_msgs__action__Pilot_GetResult_Response__rosidl_typesupport_introspection_c__Pilot_GetResult_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t cola2_msgs__action__Pilot_GetResult_Response__rosidl_typesupport_introspection_c__Pilot_GetResult_Response_message_type_support_handle = {
  0,
  &cola2_msgs__action__Pilot_GetResult_Response__rosidl_typesupport_introspection_c__Pilot_GetResult_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_cola2_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cola2_msgs, action, Pilot_GetResult_Response)() {
  cola2_msgs__action__Pilot_GetResult_Response__rosidl_typesupport_introspection_c__Pilot_GetResult_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cola2_msgs, action, Pilot_Result)();
  if (!cola2_msgs__action__Pilot_GetResult_Response__rosidl_typesupport_introspection_c__Pilot_GetResult_Response_message_type_support_handle.typesupport_identifier) {
    cola2_msgs__action__Pilot_GetResult_Response__rosidl_typesupport_introspection_c__Pilot_GetResult_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &cola2_msgs__action__Pilot_GetResult_Response__rosidl_typesupport_introspection_c__Pilot_GetResult_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "cola2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "cola2_msgs/action/detail/pilot__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers cola2_msgs__action__detail__pilot__rosidl_typesupport_introspection_c__Pilot_GetResult_service_members = {
  "cola2_msgs__action",  // service namespace
  "Pilot_GetResult",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // cola2_msgs__action__detail__pilot__rosidl_typesupport_introspection_c__Pilot_GetResult_Request_message_type_support_handle,
  NULL  // response message
  // cola2_msgs__action__detail__pilot__rosidl_typesupport_introspection_c__Pilot_GetResult_Response_message_type_support_handle
};

static rosidl_service_type_support_t cola2_msgs__action__detail__pilot__rosidl_typesupport_introspection_c__Pilot_GetResult_service_type_support_handle = {
  0,
  &cola2_msgs__action__detail__pilot__rosidl_typesupport_introspection_c__Pilot_GetResult_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cola2_msgs, action, Pilot_GetResult_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cola2_msgs, action, Pilot_GetResult_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_cola2_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cola2_msgs, action, Pilot_GetResult)() {
  if (!cola2_msgs__action__detail__pilot__rosidl_typesupport_introspection_c__Pilot_GetResult_service_type_support_handle.typesupport_identifier) {
    cola2_msgs__action__detail__pilot__rosidl_typesupport_introspection_c__Pilot_GetResult_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)cola2_msgs__action__detail__pilot__rosidl_typesupport_introspection_c__Pilot_GetResult_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cola2_msgs, action, Pilot_GetResult_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cola2_msgs, action, Pilot_GetResult_Response)()->data;
  }

  return &cola2_msgs__action__detail__pilot__rosidl_typesupport_introspection_c__Pilot_GetResult_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "cola2_msgs/action/detail/pilot__rosidl_typesupport_introspection_c.h"
// already included above
// #include "cola2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "cola2_msgs/action/detail/pilot__functions.h"
// already included above
// #include "cola2_msgs/action/detail/pilot__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `feedback`
// already included above
// #include "cola2_msgs/action/pilot.h"
// Member `feedback`
// already included above
// #include "cola2_msgs/action/detail/pilot__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void cola2_msgs__action__Pilot_FeedbackMessage__rosidl_typesupport_introspection_c__Pilot_FeedbackMessage_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  cola2_msgs__action__Pilot_FeedbackMessage__init(message_memory);
}

void cola2_msgs__action__Pilot_FeedbackMessage__rosidl_typesupport_introspection_c__Pilot_FeedbackMessage_fini_function(void * message_memory)
{
  cola2_msgs__action__Pilot_FeedbackMessage__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember cola2_msgs__action__Pilot_FeedbackMessage__rosidl_typesupport_introspection_c__Pilot_FeedbackMessage_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__action__Pilot_FeedbackMessage, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "feedback",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__action__Pilot_FeedbackMessage, feedback),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers cola2_msgs__action__Pilot_FeedbackMessage__rosidl_typesupport_introspection_c__Pilot_FeedbackMessage_message_members = {
  "cola2_msgs__action",  // message namespace
  "Pilot_FeedbackMessage",  // message name
  2,  // number of fields
  sizeof(cola2_msgs__action__Pilot_FeedbackMessage),
  cola2_msgs__action__Pilot_FeedbackMessage__rosidl_typesupport_introspection_c__Pilot_FeedbackMessage_message_member_array,  // message members
  cola2_msgs__action__Pilot_FeedbackMessage__rosidl_typesupport_introspection_c__Pilot_FeedbackMessage_init_function,  // function to initialize message memory (memory has to be allocated)
  cola2_msgs__action__Pilot_FeedbackMessage__rosidl_typesupport_introspection_c__Pilot_FeedbackMessage_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t cola2_msgs__action__Pilot_FeedbackMessage__rosidl_typesupport_introspection_c__Pilot_FeedbackMessage_message_type_support_handle = {
  0,
  &cola2_msgs__action__Pilot_FeedbackMessage__rosidl_typesupport_introspection_c__Pilot_FeedbackMessage_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_cola2_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cola2_msgs, action, Pilot_FeedbackMessage)() {
  cola2_msgs__action__Pilot_FeedbackMessage__rosidl_typesupport_introspection_c__Pilot_FeedbackMessage_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  cola2_msgs__action__Pilot_FeedbackMessage__rosidl_typesupport_introspection_c__Pilot_FeedbackMessage_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cola2_msgs, action, Pilot_Feedback)();
  if (!cola2_msgs__action__Pilot_FeedbackMessage__rosidl_typesupport_introspection_c__Pilot_FeedbackMessage_message_type_support_handle.typesupport_identifier) {
    cola2_msgs__action__Pilot_FeedbackMessage__rosidl_typesupport_introspection_c__Pilot_FeedbackMessage_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &cola2_msgs__action__Pilot_FeedbackMessage__rosidl_typesupport_introspection_c__Pilot_FeedbackMessage_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
