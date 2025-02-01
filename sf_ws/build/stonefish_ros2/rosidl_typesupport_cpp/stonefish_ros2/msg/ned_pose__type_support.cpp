// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from stonefish_ros2:msg/NEDPose.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "stonefish_ros2/msg/detail/ned_pose__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace stonefish_ros2
{

namespace msg
{

namespace rosidl_typesupport_cpp
{

typedef struct _NEDPose_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _NEDPose_type_support_ids_t;

static const _NEDPose_type_support_ids_t _NEDPose_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _NEDPose_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _NEDPose_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _NEDPose_type_support_symbol_names_t _NEDPose_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, stonefish_ros2, msg, NEDPose)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, stonefish_ros2, msg, NEDPose)),
  }
};

typedef struct _NEDPose_type_support_data_t
{
  void * data[2];
} _NEDPose_type_support_data_t;

static _NEDPose_type_support_data_t _NEDPose_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _NEDPose_message_typesupport_map = {
  2,
  "stonefish_ros2",
  &_NEDPose_message_typesupport_ids.typesupport_identifier[0],
  &_NEDPose_message_typesupport_symbol_names.symbol_name[0],
  &_NEDPose_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t NEDPose_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_NEDPose_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace msg

}  // namespace stonefish_ros2

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<stonefish_ros2::msg::NEDPose>()
{
  return &::stonefish_ros2::msg::rosidl_typesupport_cpp::NEDPose_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, stonefish_ros2, msg, NEDPose)() {
  return get_message_type_support_handle<stonefish_ros2::msg::NEDPose>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp
