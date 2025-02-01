// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from stonefish_ros2:msg/DVLBeam.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "stonefish_ros2/msg/detail/dvl_beam__rosidl_typesupport_introspection_c.h"
#include "stonefish_ros2/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "stonefish_ros2/msg/detail/dvl_beam__functions.h"
#include "stonefish_ros2/msg/detail/dvl_beam__struct.h"


// Include directives for member types
// Member `pose`
#include "geometry_msgs/msg/pose_stamped.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose_stamped__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void stonefish_ros2__msg__DVLBeam__rosidl_typesupport_introspection_c__DVLBeam_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  stonefish_ros2__msg__DVLBeam__init(message_memory);
}

void stonefish_ros2__msg__DVLBeam__rosidl_typesupport_introspection_c__DVLBeam_fini_function(void * message_memory)
{
  stonefish_ros2__msg__DVLBeam__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember stonefish_ros2__msg__DVLBeam__rosidl_typesupport_introspection_c__DVLBeam_message_member_array[5] = {
  {
    "range",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(stonefish_ros2__msg__DVLBeam, range),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "range_covariance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(stonefish_ros2__msg__DVLBeam, range_covariance),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(stonefish_ros2__msg__DVLBeam, velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "velocity_covariance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(stonefish_ros2__msg__DVLBeam, velocity_covariance),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(stonefish_ros2__msg__DVLBeam, pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers stonefish_ros2__msg__DVLBeam__rosidl_typesupport_introspection_c__DVLBeam_message_members = {
  "stonefish_ros2__msg",  // message namespace
  "DVLBeam",  // message name
  5,  // number of fields
  sizeof(stonefish_ros2__msg__DVLBeam),
  stonefish_ros2__msg__DVLBeam__rosidl_typesupport_introspection_c__DVLBeam_message_member_array,  // message members
  stonefish_ros2__msg__DVLBeam__rosidl_typesupport_introspection_c__DVLBeam_init_function,  // function to initialize message memory (memory has to be allocated)
  stonefish_ros2__msg__DVLBeam__rosidl_typesupport_introspection_c__DVLBeam_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t stonefish_ros2__msg__DVLBeam__rosidl_typesupport_introspection_c__DVLBeam_message_type_support_handle = {
  0,
  &stonefish_ros2__msg__DVLBeam__rosidl_typesupport_introspection_c__DVLBeam_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_stonefish_ros2
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, stonefish_ros2, msg, DVLBeam)() {
  stonefish_ros2__msg__DVLBeam__rosidl_typesupport_introspection_c__DVLBeam_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseStamped)();
  if (!stonefish_ros2__msg__DVLBeam__rosidl_typesupport_introspection_c__DVLBeam_message_type_support_handle.typesupport_identifier) {
    stonefish_ros2__msg__DVLBeam__rosidl_typesupport_introspection_c__DVLBeam_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &stonefish_ros2__msg__DVLBeam__rosidl_typesupport_introspection_c__DVLBeam_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
