// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from cola2_msgs:msg/BodyVelocityReq.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "cola2_msgs/msg/detail/body_velocity_req__rosidl_typesupport_introspection_c.h"
#include "cola2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "cola2_msgs/msg/detail/body_velocity_req__functions.h"
#include "cola2_msgs/msg/detail/body_velocity_req__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `goal`
#include "cola2_msgs/msg/goal_descriptor.h"
// Member `goal`
#include "cola2_msgs/msg/detail/goal_descriptor__rosidl_typesupport_introspection_c.h"
// Member `twist`
#include "geometry_msgs/msg/twist.h"
// Member `twist`
#include "geometry_msgs/msg/detail/twist__rosidl_typesupport_introspection_c.h"
// Member `disable_axis`
#include "cola2_msgs/msg/bool6_axis.h"
// Member `disable_axis`
#include "cola2_msgs/msg/detail/bool6_axis__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void cola2_msgs__msg__BodyVelocityReq__rosidl_typesupport_introspection_c__BodyVelocityReq_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  cola2_msgs__msg__BodyVelocityReq__init(message_memory);
}

void cola2_msgs__msg__BodyVelocityReq__rosidl_typesupport_introspection_c__BodyVelocityReq_fini_function(void * message_memory)
{
  cola2_msgs__msg__BodyVelocityReq__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember cola2_msgs__msg__BodyVelocityReq__rosidl_typesupport_introspection_c__BodyVelocityReq_message_member_array[4] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__msg__BodyVelocityReq, header),  // bytes offset in struct
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
    offsetof(cola2_msgs__msg__BodyVelocityReq, goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "twist",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__msg__BodyVelocityReq, twist),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "disable_axis",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__msg__BodyVelocityReq, disable_axis),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers cola2_msgs__msg__BodyVelocityReq__rosidl_typesupport_introspection_c__BodyVelocityReq_message_members = {
  "cola2_msgs__msg",  // message namespace
  "BodyVelocityReq",  // message name
  4,  // number of fields
  sizeof(cola2_msgs__msg__BodyVelocityReq),
  cola2_msgs__msg__BodyVelocityReq__rosidl_typesupport_introspection_c__BodyVelocityReq_message_member_array,  // message members
  cola2_msgs__msg__BodyVelocityReq__rosidl_typesupport_introspection_c__BodyVelocityReq_init_function,  // function to initialize message memory (memory has to be allocated)
  cola2_msgs__msg__BodyVelocityReq__rosidl_typesupport_introspection_c__BodyVelocityReq_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t cola2_msgs__msg__BodyVelocityReq__rosidl_typesupport_introspection_c__BodyVelocityReq_message_type_support_handle = {
  0,
  &cola2_msgs__msg__BodyVelocityReq__rosidl_typesupport_introspection_c__BodyVelocityReq_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_cola2_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cola2_msgs, msg, BodyVelocityReq)() {
  cola2_msgs__msg__BodyVelocityReq__rosidl_typesupport_introspection_c__BodyVelocityReq_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  cola2_msgs__msg__BodyVelocityReq__rosidl_typesupport_introspection_c__BodyVelocityReq_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cola2_msgs, msg, GoalDescriptor)();
  cola2_msgs__msg__BodyVelocityReq__rosidl_typesupport_introspection_c__BodyVelocityReq_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Twist)();
  cola2_msgs__msg__BodyVelocityReq__rosidl_typesupport_introspection_c__BodyVelocityReq_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cola2_msgs, msg, Bool6Axis)();
  if (!cola2_msgs__msg__BodyVelocityReq__rosidl_typesupport_introspection_c__BodyVelocityReq_message_type_support_handle.typesupport_identifier) {
    cola2_msgs__msg__BodyVelocityReq__rosidl_typesupport_introspection_c__BodyVelocityReq_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &cola2_msgs__msg__BodyVelocityReq__rosidl_typesupport_introspection_c__BodyVelocityReq_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
