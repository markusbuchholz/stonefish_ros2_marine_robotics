// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from cola2_msgs:srv/MaxJoyVelocity.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "cola2_msgs/srv/detail/max_joy_velocity__rosidl_typesupport_introspection_c.h"
#include "cola2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "cola2_msgs/srv/detail/max_joy_velocity__functions.h"
#include "cola2_msgs/srv/detail/max_joy_velocity__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void cola2_msgs__srv__MaxJoyVelocity_Request__rosidl_typesupport_introspection_c__MaxJoyVelocity_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  cola2_msgs__srv__MaxJoyVelocity_Request__init(message_memory);
}

void cola2_msgs__srv__MaxJoyVelocity_Request__rosidl_typesupport_introspection_c__MaxJoyVelocity_Request_fini_function(void * message_memory)
{
  cola2_msgs__srv__MaxJoyVelocity_Request__fini(message_memory);
}

size_t cola2_msgs__srv__MaxJoyVelocity_Request__rosidl_typesupport_introspection_c__size_function__MaxJoyVelocity_Request__max_joy_velocity(
  const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * cola2_msgs__srv__MaxJoyVelocity_Request__rosidl_typesupport_introspection_c__get_const_function__MaxJoyVelocity_Request__max_joy_velocity(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * cola2_msgs__srv__MaxJoyVelocity_Request__rosidl_typesupport_introspection_c__get_function__MaxJoyVelocity_Request__max_joy_velocity(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void cola2_msgs__srv__MaxJoyVelocity_Request__rosidl_typesupport_introspection_c__fetch_function__MaxJoyVelocity_Request__max_joy_velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    cola2_msgs__srv__MaxJoyVelocity_Request__rosidl_typesupport_introspection_c__get_const_function__MaxJoyVelocity_Request__max_joy_velocity(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void cola2_msgs__srv__MaxJoyVelocity_Request__rosidl_typesupport_introspection_c__assign_function__MaxJoyVelocity_Request__max_joy_velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    cola2_msgs__srv__MaxJoyVelocity_Request__rosidl_typesupport_introspection_c__get_function__MaxJoyVelocity_Request__max_joy_velocity(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember cola2_msgs__srv__MaxJoyVelocity_Request__rosidl_typesupport_introspection_c__MaxJoyVelocity_Request_message_member_array[1] = {
  {
    "max_joy_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__srv__MaxJoyVelocity_Request, max_joy_velocity),  // bytes offset in struct
    NULL,  // default value
    cola2_msgs__srv__MaxJoyVelocity_Request__rosidl_typesupport_introspection_c__size_function__MaxJoyVelocity_Request__max_joy_velocity,  // size() function pointer
    cola2_msgs__srv__MaxJoyVelocity_Request__rosidl_typesupport_introspection_c__get_const_function__MaxJoyVelocity_Request__max_joy_velocity,  // get_const(index) function pointer
    cola2_msgs__srv__MaxJoyVelocity_Request__rosidl_typesupport_introspection_c__get_function__MaxJoyVelocity_Request__max_joy_velocity,  // get(index) function pointer
    cola2_msgs__srv__MaxJoyVelocity_Request__rosidl_typesupport_introspection_c__fetch_function__MaxJoyVelocity_Request__max_joy_velocity,  // fetch(index, &value) function pointer
    cola2_msgs__srv__MaxJoyVelocity_Request__rosidl_typesupport_introspection_c__assign_function__MaxJoyVelocity_Request__max_joy_velocity,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers cola2_msgs__srv__MaxJoyVelocity_Request__rosidl_typesupport_introspection_c__MaxJoyVelocity_Request_message_members = {
  "cola2_msgs__srv",  // message namespace
  "MaxJoyVelocity_Request",  // message name
  1,  // number of fields
  sizeof(cola2_msgs__srv__MaxJoyVelocity_Request),
  cola2_msgs__srv__MaxJoyVelocity_Request__rosidl_typesupport_introspection_c__MaxJoyVelocity_Request_message_member_array,  // message members
  cola2_msgs__srv__MaxJoyVelocity_Request__rosidl_typesupport_introspection_c__MaxJoyVelocity_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  cola2_msgs__srv__MaxJoyVelocity_Request__rosidl_typesupport_introspection_c__MaxJoyVelocity_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t cola2_msgs__srv__MaxJoyVelocity_Request__rosidl_typesupport_introspection_c__MaxJoyVelocity_Request_message_type_support_handle = {
  0,
  &cola2_msgs__srv__MaxJoyVelocity_Request__rosidl_typesupport_introspection_c__MaxJoyVelocity_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_cola2_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cola2_msgs, srv, MaxJoyVelocity_Request)() {
  if (!cola2_msgs__srv__MaxJoyVelocity_Request__rosidl_typesupport_introspection_c__MaxJoyVelocity_Request_message_type_support_handle.typesupport_identifier) {
    cola2_msgs__srv__MaxJoyVelocity_Request__rosidl_typesupport_introspection_c__MaxJoyVelocity_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &cola2_msgs__srv__MaxJoyVelocity_Request__rosidl_typesupport_introspection_c__MaxJoyVelocity_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "cola2_msgs/srv/detail/max_joy_velocity__rosidl_typesupport_introspection_c.h"
// already included above
// #include "cola2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "cola2_msgs/srv/detail/max_joy_velocity__functions.h"
// already included above
// #include "cola2_msgs/srv/detail/max_joy_velocity__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void cola2_msgs__srv__MaxJoyVelocity_Response__rosidl_typesupport_introspection_c__MaxJoyVelocity_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  cola2_msgs__srv__MaxJoyVelocity_Response__init(message_memory);
}

void cola2_msgs__srv__MaxJoyVelocity_Response__rosidl_typesupport_introspection_c__MaxJoyVelocity_Response_fini_function(void * message_memory)
{
  cola2_msgs__srv__MaxJoyVelocity_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember cola2_msgs__srv__MaxJoyVelocity_Response__rosidl_typesupport_introspection_c__MaxJoyVelocity_Response_message_member_array[1] = {
  {
    "attempted",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__srv__MaxJoyVelocity_Response, attempted),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers cola2_msgs__srv__MaxJoyVelocity_Response__rosidl_typesupport_introspection_c__MaxJoyVelocity_Response_message_members = {
  "cola2_msgs__srv",  // message namespace
  "MaxJoyVelocity_Response",  // message name
  1,  // number of fields
  sizeof(cola2_msgs__srv__MaxJoyVelocity_Response),
  cola2_msgs__srv__MaxJoyVelocity_Response__rosidl_typesupport_introspection_c__MaxJoyVelocity_Response_message_member_array,  // message members
  cola2_msgs__srv__MaxJoyVelocity_Response__rosidl_typesupport_introspection_c__MaxJoyVelocity_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  cola2_msgs__srv__MaxJoyVelocity_Response__rosidl_typesupport_introspection_c__MaxJoyVelocity_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t cola2_msgs__srv__MaxJoyVelocity_Response__rosidl_typesupport_introspection_c__MaxJoyVelocity_Response_message_type_support_handle = {
  0,
  &cola2_msgs__srv__MaxJoyVelocity_Response__rosidl_typesupport_introspection_c__MaxJoyVelocity_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_cola2_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cola2_msgs, srv, MaxJoyVelocity_Response)() {
  if (!cola2_msgs__srv__MaxJoyVelocity_Response__rosidl_typesupport_introspection_c__MaxJoyVelocity_Response_message_type_support_handle.typesupport_identifier) {
    cola2_msgs__srv__MaxJoyVelocity_Response__rosidl_typesupport_introspection_c__MaxJoyVelocity_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &cola2_msgs__srv__MaxJoyVelocity_Response__rosidl_typesupport_introspection_c__MaxJoyVelocity_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "cola2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "cola2_msgs/srv/detail/max_joy_velocity__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers cola2_msgs__srv__detail__max_joy_velocity__rosidl_typesupport_introspection_c__MaxJoyVelocity_service_members = {
  "cola2_msgs__srv",  // service namespace
  "MaxJoyVelocity",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // cola2_msgs__srv__detail__max_joy_velocity__rosidl_typesupport_introspection_c__MaxJoyVelocity_Request_message_type_support_handle,
  NULL  // response message
  // cola2_msgs__srv__detail__max_joy_velocity__rosidl_typesupport_introspection_c__MaxJoyVelocity_Response_message_type_support_handle
};

static rosidl_service_type_support_t cola2_msgs__srv__detail__max_joy_velocity__rosidl_typesupport_introspection_c__MaxJoyVelocity_service_type_support_handle = {
  0,
  &cola2_msgs__srv__detail__max_joy_velocity__rosidl_typesupport_introspection_c__MaxJoyVelocity_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cola2_msgs, srv, MaxJoyVelocity_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cola2_msgs, srv, MaxJoyVelocity_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_cola2_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cola2_msgs, srv, MaxJoyVelocity)() {
  if (!cola2_msgs__srv__detail__max_joy_velocity__rosidl_typesupport_introspection_c__MaxJoyVelocity_service_type_support_handle.typesupport_identifier) {
    cola2_msgs__srv__detail__max_joy_velocity__rosidl_typesupport_introspection_c__MaxJoyVelocity_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)cola2_msgs__srv__detail__max_joy_velocity__rosidl_typesupport_introspection_c__MaxJoyVelocity_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cola2_msgs, srv, MaxJoyVelocity_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cola2_msgs, srv, MaxJoyVelocity_Response)()->data;
  }

  return &cola2_msgs__srv__detail__max_joy_velocity__rosidl_typesupport_introspection_c__MaxJoyVelocity_service_type_support_handle;
}
