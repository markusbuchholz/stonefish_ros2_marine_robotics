// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from cola2_msgs:msg/WorldWaypointReq.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "cola2_msgs/msg/detail/world_waypoint_req__rosidl_typesupport_introspection_c.h"
#include "cola2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "cola2_msgs/msg/detail/world_waypoint_req__functions.h"
#include "cola2_msgs/msg/detail/world_waypoint_req__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `goal`
#include "cola2_msgs/msg/goal_descriptor.h"
// Member `goal`
#include "cola2_msgs/msg/detail/goal_descriptor__rosidl_typesupport_introspection_c.h"
// Member `position`
#include "cola2_msgs/msg/ned.h"
// Member `position`
#include "cola2_msgs/msg/detail/ned__rosidl_typesupport_introspection_c.h"
// Member `orientation`
// Member `orientation_tolerance`
#include "cola2_msgs/msg/rpy.h"
// Member `orientation`
// Member `orientation_tolerance`
#include "cola2_msgs/msg/detail/rpy__rosidl_typesupport_introspection_c.h"
// Member `disable_axis`
#include "cola2_msgs/msg/bool6_axis.h"
// Member `disable_axis`
#include "cola2_msgs/msg/detail/bool6_axis__rosidl_typesupport_introspection_c.h"
// Member `position_tolerance`
#include "geometry_msgs/msg/vector3.h"
// Member `position_tolerance`
#include "geometry_msgs/msg/detail/vector3__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void cola2_msgs__msg__WorldWaypointReq__rosidl_typesupport_introspection_c__WorldWaypointReq_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  cola2_msgs__msg__WorldWaypointReq__init(message_memory);
}

void cola2_msgs__msg__WorldWaypointReq__rosidl_typesupport_introspection_c__WorldWaypointReq_fini_function(void * message_memory)
{
  cola2_msgs__msg__WorldWaypointReq__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember cola2_msgs__msg__WorldWaypointReq__rosidl_typesupport_introspection_c__WorldWaypointReq_message_member_array[9] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__msg__WorldWaypointReq, header),  // bytes offset in struct
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
    offsetof(cola2_msgs__msg__WorldWaypointReq, goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "altitude_mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__msg__WorldWaypointReq, altitude_mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__msg__WorldWaypointReq, position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "altitude",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__msg__WorldWaypointReq, altitude),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "orientation",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__msg__WorldWaypointReq, orientation),  // bytes offset in struct
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
    offsetof(cola2_msgs__msg__WorldWaypointReq, disable_axis),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position_tolerance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__msg__WorldWaypointReq, position_tolerance),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "orientation_tolerance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs__msg__WorldWaypointReq, orientation_tolerance),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers cola2_msgs__msg__WorldWaypointReq__rosidl_typesupport_introspection_c__WorldWaypointReq_message_members = {
  "cola2_msgs__msg",  // message namespace
  "WorldWaypointReq",  // message name
  9,  // number of fields
  sizeof(cola2_msgs__msg__WorldWaypointReq),
  cola2_msgs__msg__WorldWaypointReq__rosidl_typesupport_introspection_c__WorldWaypointReq_message_member_array,  // message members
  cola2_msgs__msg__WorldWaypointReq__rosidl_typesupport_introspection_c__WorldWaypointReq_init_function,  // function to initialize message memory (memory has to be allocated)
  cola2_msgs__msg__WorldWaypointReq__rosidl_typesupport_introspection_c__WorldWaypointReq_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t cola2_msgs__msg__WorldWaypointReq__rosidl_typesupport_introspection_c__WorldWaypointReq_message_type_support_handle = {
  0,
  &cola2_msgs__msg__WorldWaypointReq__rosidl_typesupport_introspection_c__WorldWaypointReq_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_cola2_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cola2_msgs, msg, WorldWaypointReq)() {
  cola2_msgs__msg__WorldWaypointReq__rosidl_typesupport_introspection_c__WorldWaypointReq_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  cola2_msgs__msg__WorldWaypointReq__rosidl_typesupport_introspection_c__WorldWaypointReq_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cola2_msgs, msg, GoalDescriptor)();
  cola2_msgs__msg__WorldWaypointReq__rosidl_typesupport_introspection_c__WorldWaypointReq_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cola2_msgs, msg, NED)();
  cola2_msgs__msg__WorldWaypointReq__rosidl_typesupport_introspection_c__WorldWaypointReq_message_member_array[5].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cola2_msgs, msg, RPY)();
  cola2_msgs__msg__WorldWaypointReq__rosidl_typesupport_introspection_c__WorldWaypointReq_message_member_array[6].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cola2_msgs, msg, Bool6Axis)();
  cola2_msgs__msg__WorldWaypointReq__rosidl_typesupport_introspection_c__WorldWaypointReq_message_member_array[7].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  cola2_msgs__msg__WorldWaypointReq__rosidl_typesupport_introspection_c__WorldWaypointReq_message_member_array[8].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cola2_msgs, msg, RPY)();
  if (!cola2_msgs__msg__WorldWaypointReq__rosidl_typesupport_introspection_c__WorldWaypointReq_message_type_support_handle.typesupport_identifier) {
    cola2_msgs__msg__WorldWaypointReq__rosidl_typesupport_introspection_c__WorldWaypointReq_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &cola2_msgs__msg__WorldWaypointReq__rosidl_typesupport_introspection_c__WorldWaypointReq_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
