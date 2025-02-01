// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from stonefish_ros2:msg/ThrusterState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "stonefish_ros2/msg/detail/thruster_state__rosidl_typesupport_introspection_c.h"
#include "stonefish_ros2/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "stonefish_ros2/msg/detail/thruster_state__functions.h"
#include "stonefish_ros2/msg/detail/thruster_state__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `setpoint`
// Member `rpm`
// Member `thrust`
// Member `torque`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__ThrusterState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  stonefish_ros2__msg__ThrusterState__init(message_memory);
}

void stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__ThrusterState_fini_function(void * message_memory)
{
  stonefish_ros2__msg__ThrusterState__fini(message_memory);
}

size_t stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__size_function__ThrusterState__setpoint(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__get_const_function__ThrusterState__setpoint(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__get_function__ThrusterState__setpoint(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__fetch_function__ThrusterState__setpoint(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__get_const_function__ThrusterState__setpoint(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__assign_function__ThrusterState__setpoint(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__get_function__ThrusterState__setpoint(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__resize_function__ThrusterState__setpoint(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__size_function__ThrusterState__rpm(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__get_const_function__ThrusterState__rpm(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__get_function__ThrusterState__rpm(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__fetch_function__ThrusterState__rpm(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__get_const_function__ThrusterState__rpm(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__assign_function__ThrusterState__rpm(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__get_function__ThrusterState__rpm(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__resize_function__ThrusterState__rpm(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__size_function__ThrusterState__thrust(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__get_const_function__ThrusterState__thrust(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__get_function__ThrusterState__thrust(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__fetch_function__ThrusterState__thrust(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__get_const_function__ThrusterState__thrust(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__assign_function__ThrusterState__thrust(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__get_function__ThrusterState__thrust(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__resize_function__ThrusterState__thrust(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__size_function__ThrusterState__torque(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__get_const_function__ThrusterState__torque(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__get_function__ThrusterState__torque(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__fetch_function__ThrusterState__torque(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__get_const_function__ThrusterState__torque(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__assign_function__ThrusterState__torque(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__get_function__ThrusterState__torque(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__resize_function__ThrusterState__torque(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__ThrusterState_message_member_array[5] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(stonefish_ros2__msg__ThrusterState, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "setpoint",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(stonefish_ros2__msg__ThrusterState, setpoint),  // bytes offset in struct
    NULL,  // default value
    stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__size_function__ThrusterState__setpoint,  // size() function pointer
    stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__get_const_function__ThrusterState__setpoint,  // get_const(index) function pointer
    stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__get_function__ThrusterState__setpoint,  // get(index) function pointer
    stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__fetch_function__ThrusterState__setpoint,  // fetch(index, &value) function pointer
    stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__assign_function__ThrusterState__setpoint,  // assign(index, value) function pointer
    stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__resize_function__ThrusterState__setpoint  // resize(index) function pointer
  },
  {
    "rpm",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(stonefish_ros2__msg__ThrusterState, rpm),  // bytes offset in struct
    NULL,  // default value
    stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__size_function__ThrusterState__rpm,  // size() function pointer
    stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__get_const_function__ThrusterState__rpm,  // get_const(index) function pointer
    stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__get_function__ThrusterState__rpm,  // get(index) function pointer
    stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__fetch_function__ThrusterState__rpm,  // fetch(index, &value) function pointer
    stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__assign_function__ThrusterState__rpm,  // assign(index, value) function pointer
    stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__resize_function__ThrusterState__rpm  // resize(index) function pointer
  },
  {
    "thrust",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(stonefish_ros2__msg__ThrusterState, thrust),  // bytes offset in struct
    NULL,  // default value
    stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__size_function__ThrusterState__thrust,  // size() function pointer
    stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__get_const_function__ThrusterState__thrust,  // get_const(index) function pointer
    stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__get_function__ThrusterState__thrust,  // get(index) function pointer
    stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__fetch_function__ThrusterState__thrust,  // fetch(index, &value) function pointer
    stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__assign_function__ThrusterState__thrust,  // assign(index, value) function pointer
    stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__resize_function__ThrusterState__thrust  // resize(index) function pointer
  },
  {
    "torque",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(stonefish_ros2__msg__ThrusterState, torque),  // bytes offset in struct
    NULL,  // default value
    stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__size_function__ThrusterState__torque,  // size() function pointer
    stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__get_const_function__ThrusterState__torque,  // get_const(index) function pointer
    stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__get_function__ThrusterState__torque,  // get(index) function pointer
    stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__fetch_function__ThrusterState__torque,  // fetch(index, &value) function pointer
    stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__assign_function__ThrusterState__torque,  // assign(index, value) function pointer
    stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__resize_function__ThrusterState__torque  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__ThrusterState_message_members = {
  "stonefish_ros2__msg",  // message namespace
  "ThrusterState",  // message name
  5,  // number of fields
  sizeof(stonefish_ros2__msg__ThrusterState),
  stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__ThrusterState_message_member_array,  // message members
  stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__ThrusterState_init_function,  // function to initialize message memory (memory has to be allocated)
  stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__ThrusterState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__ThrusterState_message_type_support_handle = {
  0,
  &stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__ThrusterState_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_stonefish_ros2
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, stonefish_ros2, msg, ThrusterState)() {
  stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__ThrusterState_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__ThrusterState_message_type_support_handle.typesupport_identifier) {
    stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__ThrusterState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &stonefish_ros2__msg__ThrusterState__rosidl_typesupport_introspection_c__ThrusterState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
