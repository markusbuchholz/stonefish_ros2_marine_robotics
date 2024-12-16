// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from cola2_msgs:msg/SafetySupervisorStatus.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "cola2_msgs/msg/detail/safety_supervisor_status__rosidl_typesupport_introspection_c.h"
#include "cola2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "cola2_msgs/msg/detail/safety_supervisor_status__functions.h"
#include "cola2_msgs/msg/detail/safety_supervisor_status__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `recovery_action`
#include "cola2_msgs/msg/recovery_action.h"
// Member `recovery_action`
#include "cola2_msgs/msg/detail/recovery_action__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void cola2_msgs__msg__SafetySupervisorStatus__rosidl_typesupport_introspection_c__SafetySupervisorStatus_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  cola2_msgs__msg__SafetySupervisorStatus__init(message_memory);
}

void cola2_msgs__msg__SafetySupervisorStatus__rosidl_typesupport_introspection_c__SafetySupervisorStatus_fini_function(void * message_memory)
{
  cola2_msgs__msg__SafetySupervisorStatus__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember cola2_msgs__msg__SafetySupervisorStatus__rosidl_typesupport_introspection_c__SafetySupervisorStatus_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__msg__SafetySupervisorStatus, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "status_code",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__msg__SafetySupervisorStatus, status_code),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "recovery_action",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__msg__SafetySupervisorStatus, recovery_action),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers cola2_msgs__msg__SafetySupervisorStatus__rosidl_typesupport_introspection_c__SafetySupervisorStatus_message_members = {
  "cola2_msgs__msg",  // message namespace
  "SafetySupervisorStatus",  // message name
  3,  // number of fields
  sizeof(cola2_msgs__msg__SafetySupervisorStatus),
  cola2_msgs__msg__SafetySupervisorStatus__rosidl_typesupport_introspection_c__SafetySupervisorStatus_message_member_array,  // message members
  cola2_msgs__msg__SafetySupervisorStatus__rosidl_typesupport_introspection_c__SafetySupervisorStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  cola2_msgs__msg__SafetySupervisorStatus__rosidl_typesupport_introspection_c__SafetySupervisorStatus_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t cola2_msgs__msg__SafetySupervisorStatus__rosidl_typesupport_introspection_c__SafetySupervisorStatus_message_type_support_handle = {
  0,
  &cola2_msgs__msg__SafetySupervisorStatus__rosidl_typesupport_introspection_c__SafetySupervisorStatus_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_cola2_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cola2_msgs, msg, SafetySupervisorStatus)() {
  cola2_msgs__msg__SafetySupervisorStatus__rosidl_typesupport_introspection_c__SafetySupervisorStatus_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  cola2_msgs__msg__SafetySupervisorStatus__rosidl_typesupport_introspection_c__SafetySupervisorStatus_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cola2_msgs, msg, RecoveryAction)();
  if (!cola2_msgs__msg__SafetySupervisorStatus__rosidl_typesupport_introspection_c__SafetySupervisorStatus_message_type_support_handle.typesupport_identifier) {
    cola2_msgs__msg__SafetySupervisorStatus__rosidl_typesupport_introspection_c__SafetySupervisorStatus_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &cola2_msgs__msg__SafetySupervisorStatus__rosidl_typesupport_introspection_c__SafetySupervisorStatus_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
