// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from cola2_msgs:msg/CaptainStateFeedback.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "cola2_msgs/msg/detail/captain_state_feedback__rosidl_typesupport_introspection_c.h"
#include "cola2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "cola2_msgs/msg/detail/captain_state_feedback__functions.h"
#include "cola2_msgs/msg/detail/captain_state_feedback__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `name`
#include "rosidl_runtime_c/string_functions.h"
// Member `keyvalues`
#include "cola2_msgs/msg/key_value.h"
// Member `keyvalues`
#include "cola2_msgs/msg/detail/key_value__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void cola2_msgs__msg__CaptainStateFeedback__rosidl_typesupport_introspection_c__CaptainStateFeedback_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  cola2_msgs__msg__CaptainStateFeedback__init(message_memory);
}

void cola2_msgs__msg__CaptainStateFeedback__rosidl_typesupport_introspection_c__CaptainStateFeedback_fini_function(void * message_memory)
{
  cola2_msgs__msg__CaptainStateFeedback__fini(message_memory);
}

size_t cola2_msgs__msg__CaptainStateFeedback__rosidl_typesupport_introspection_c__size_function__CaptainStateFeedback__keyvalues(
  const void * untyped_member)
{
  const cola2_msgs__msg__KeyValue__Sequence * member =
    (const cola2_msgs__msg__KeyValue__Sequence *)(untyped_member);
  return member->size;
}

const void * cola2_msgs__msg__CaptainStateFeedback__rosidl_typesupport_introspection_c__get_const_function__CaptainStateFeedback__keyvalues(
  const void * untyped_member, size_t index)
{
  const cola2_msgs__msg__KeyValue__Sequence * member =
    (const cola2_msgs__msg__KeyValue__Sequence *)(untyped_member);
  return &member->data[index];
}

void * cola2_msgs__msg__CaptainStateFeedback__rosidl_typesupport_introspection_c__get_function__CaptainStateFeedback__keyvalues(
  void * untyped_member, size_t index)
{
  cola2_msgs__msg__KeyValue__Sequence * member =
    (cola2_msgs__msg__KeyValue__Sequence *)(untyped_member);
  return &member->data[index];
}

void cola2_msgs__msg__CaptainStateFeedback__rosidl_typesupport_introspection_c__fetch_function__CaptainStateFeedback__keyvalues(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const cola2_msgs__msg__KeyValue * item =
    ((const cola2_msgs__msg__KeyValue *)
    cola2_msgs__msg__CaptainStateFeedback__rosidl_typesupport_introspection_c__get_const_function__CaptainStateFeedback__keyvalues(untyped_member, index));
  cola2_msgs__msg__KeyValue * value =
    (cola2_msgs__msg__KeyValue *)(untyped_value);
  *value = *item;
}

void cola2_msgs__msg__CaptainStateFeedback__rosidl_typesupport_introspection_c__assign_function__CaptainStateFeedback__keyvalues(
  void * untyped_member, size_t index, const void * untyped_value)
{
  cola2_msgs__msg__KeyValue * item =
    ((cola2_msgs__msg__KeyValue *)
    cola2_msgs__msg__CaptainStateFeedback__rosidl_typesupport_introspection_c__get_function__CaptainStateFeedback__keyvalues(untyped_member, index));
  const cola2_msgs__msg__KeyValue * value =
    (const cola2_msgs__msg__KeyValue *)(untyped_value);
  *item = *value;
}

bool cola2_msgs__msg__CaptainStateFeedback__rosidl_typesupport_introspection_c__resize_function__CaptainStateFeedback__keyvalues(
  void * untyped_member, size_t size)
{
  cola2_msgs__msg__KeyValue__Sequence * member =
    (cola2_msgs__msg__KeyValue__Sequence *)(untyped_member);
  cola2_msgs__msg__KeyValue__Sequence__fini(member);
  return cola2_msgs__msg__KeyValue__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember cola2_msgs__msg__CaptainStateFeedback__rosidl_typesupport_introspection_c__CaptainStateFeedback_message_member_array[6] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__msg__CaptainStateFeedback, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__msg__CaptainStateFeedback, state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "time_to_finish",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__msg__CaptainStateFeedback, time_to_finish),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__msg__CaptainStateFeedback, name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__msg__CaptainStateFeedback, id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "keyvalues",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__msg__CaptainStateFeedback, keyvalues),  // bytes offset in struct
    NULL,  // default value
    cola2_msgs__msg__CaptainStateFeedback__rosidl_typesupport_introspection_c__size_function__CaptainStateFeedback__keyvalues,  // size() function pointer
    cola2_msgs__msg__CaptainStateFeedback__rosidl_typesupport_introspection_c__get_const_function__CaptainStateFeedback__keyvalues,  // get_const(index) function pointer
    cola2_msgs__msg__CaptainStateFeedback__rosidl_typesupport_introspection_c__get_function__CaptainStateFeedback__keyvalues,  // get(index) function pointer
    cola2_msgs__msg__CaptainStateFeedback__rosidl_typesupport_introspection_c__fetch_function__CaptainStateFeedback__keyvalues,  // fetch(index, &value) function pointer
    cola2_msgs__msg__CaptainStateFeedback__rosidl_typesupport_introspection_c__assign_function__CaptainStateFeedback__keyvalues,  // assign(index, value) function pointer
    cola2_msgs__msg__CaptainStateFeedback__rosidl_typesupport_introspection_c__resize_function__CaptainStateFeedback__keyvalues  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers cola2_msgs__msg__CaptainStateFeedback__rosidl_typesupport_introspection_c__CaptainStateFeedback_message_members = {
  "cola2_msgs__msg",  // message namespace
  "CaptainStateFeedback",  // message name
  6,  // number of fields
  sizeof(cola2_msgs__msg__CaptainStateFeedback),
  cola2_msgs__msg__CaptainStateFeedback__rosidl_typesupport_introspection_c__CaptainStateFeedback_message_member_array,  // message members
  cola2_msgs__msg__CaptainStateFeedback__rosidl_typesupport_introspection_c__CaptainStateFeedback_init_function,  // function to initialize message memory (memory has to be allocated)
  cola2_msgs__msg__CaptainStateFeedback__rosidl_typesupport_introspection_c__CaptainStateFeedback_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t cola2_msgs__msg__CaptainStateFeedback__rosidl_typesupport_introspection_c__CaptainStateFeedback_message_type_support_handle = {
  0,
  &cola2_msgs__msg__CaptainStateFeedback__rosidl_typesupport_introspection_c__CaptainStateFeedback_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_cola2_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cola2_msgs, msg, CaptainStateFeedback)() {
  cola2_msgs__msg__CaptainStateFeedback__rosidl_typesupport_introspection_c__CaptainStateFeedback_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  cola2_msgs__msg__CaptainStateFeedback__rosidl_typesupport_introspection_c__CaptainStateFeedback_message_member_array[5].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cola2_msgs, msg, KeyValue)();
  if (!cola2_msgs__msg__CaptainStateFeedback__rosidl_typesupport_introspection_c__CaptainStateFeedback_message_type_support_handle.typesupport_identifier) {
    cola2_msgs__msg__CaptainStateFeedback__rosidl_typesupport_introspection_c__CaptainStateFeedback_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &cola2_msgs__msg__CaptainStateFeedback__rosidl_typesupport_introspection_c__CaptainStateFeedback_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
