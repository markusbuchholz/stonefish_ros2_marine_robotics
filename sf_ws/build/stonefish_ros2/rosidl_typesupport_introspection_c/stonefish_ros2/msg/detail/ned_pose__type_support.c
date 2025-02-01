// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from stonefish_ros2:msg/NEDPose.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "stonefish_ros2/msg/detail/ned_pose__rosidl_typesupport_introspection_c.h"
#include "stonefish_ros2/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "stonefish_ros2/msg/detail/ned_pose__functions.h"
#include "stonefish_ros2/msg/detail/ned_pose__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void stonefish_ros2__msg__NEDPose__rosidl_typesupport_introspection_c__NEDPose_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  stonefish_ros2__msg__NEDPose__init(message_memory);
}

void stonefish_ros2__msg__NEDPose__rosidl_typesupport_introspection_c__NEDPose_fini_function(void * message_memory)
{
  stonefish_ros2__msg__NEDPose__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember stonefish_ros2__msg__NEDPose__rosidl_typesupport_introspection_c__NEDPose_message_member_array[6] = {
  {
    "north",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(stonefish_ros2__msg__NEDPose, north),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "east",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(stonefish_ros2__msg__NEDPose, east),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "down",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(stonefish_ros2__msg__NEDPose, down),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "roll",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(stonefish_ros2__msg__NEDPose, roll),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pitch",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(stonefish_ros2__msg__NEDPose, pitch),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "yaw",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(stonefish_ros2__msg__NEDPose, yaw),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers stonefish_ros2__msg__NEDPose__rosidl_typesupport_introspection_c__NEDPose_message_members = {
  "stonefish_ros2__msg",  // message namespace
  "NEDPose",  // message name
  6,  // number of fields
  sizeof(stonefish_ros2__msg__NEDPose),
  stonefish_ros2__msg__NEDPose__rosidl_typesupport_introspection_c__NEDPose_message_member_array,  // message members
  stonefish_ros2__msg__NEDPose__rosidl_typesupport_introspection_c__NEDPose_init_function,  // function to initialize message memory (memory has to be allocated)
  stonefish_ros2__msg__NEDPose__rosidl_typesupport_introspection_c__NEDPose_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t stonefish_ros2__msg__NEDPose__rosidl_typesupport_introspection_c__NEDPose_message_type_support_handle = {
  0,
  &stonefish_ros2__msg__NEDPose__rosidl_typesupport_introspection_c__NEDPose_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_stonefish_ros2
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, stonefish_ros2, msg, NEDPose)() {
  if (!stonefish_ros2__msg__NEDPose__rosidl_typesupport_introspection_c__NEDPose_message_type_support_handle.typesupport_identifier) {
    stonefish_ros2__msg__NEDPose__rosidl_typesupport_introspection_c__NEDPose_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &stonefish_ros2__msg__NEDPose__rosidl_typesupport_introspection_c__NEDPose_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
