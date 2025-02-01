// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from cola2_msgs:msg/WorldWaypointReq.idl
// generated code does not contain a copyright notice
#include "cola2_msgs/msg/detail/world_waypoint_req__rosidl_typesupport_fastrtps_cpp.hpp"
#include "cola2_msgs/msg/detail/world_waypoint_req__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace std_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const std_msgs::msg::Header &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  std_msgs::msg::Header &);
size_t get_serialized_size(
  const std_msgs::msg::Header &,
  size_t current_alignment);
size_t
max_serialized_size_Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace std_msgs

namespace cola2_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const cola2_msgs::msg::GoalDescriptor &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  cola2_msgs::msg::GoalDescriptor &);
size_t get_serialized_size(
  const cola2_msgs::msg::GoalDescriptor &,
  size_t current_alignment);
size_t
max_serialized_size_GoalDescriptor(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace cola2_msgs

namespace cola2_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const cola2_msgs::msg::NED &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  cola2_msgs::msg::NED &);
size_t get_serialized_size(
  const cola2_msgs::msg::NED &,
  size_t current_alignment);
size_t
max_serialized_size_NED(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace cola2_msgs

namespace cola2_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const cola2_msgs::msg::RPY &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  cola2_msgs::msg::RPY &);
size_t get_serialized_size(
  const cola2_msgs::msg::RPY &,
  size_t current_alignment);
size_t
max_serialized_size_RPY(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace cola2_msgs

namespace cola2_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const cola2_msgs::msg::Bool6Axis &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  cola2_msgs::msg::Bool6Axis &);
size_t get_serialized_size(
  const cola2_msgs::msg::Bool6Axis &,
  size_t current_alignment);
size_t
max_serialized_size_Bool6Axis(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace cola2_msgs

namespace geometry_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const geometry_msgs::msg::Vector3 &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  geometry_msgs::msg::Vector3 &);
size_t get_serialized_size(
  const geometry_msgs::msg::Vector3 &,
  size_t current_alignment);
size_t
max_serialized_size_Vector3(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace geometry_msgs

// functions for cola2_msgs::msg::RPY already declared above


namespace cola2_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_cola2_msgs
cdr_serialize(
  const cola2_msgs::msg::WorldWaypointReq & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.header,
    cdr);
  // Member: goal
  cola2_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.goal,
    cdr);
  // Member: altitude_mode
  cdr << (ros_message.altitude_mode ? true : false);
  // Member: position
  cola2_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.position,
    cdr);
  // Member: altitude
  cdr << ros_message.altitude;
  // Member: orientation
  cola2_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.orientation,
    cdr);
  // Member: disable_axis
  cola2_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.disable_axis,
    cdr);
  // Member: position_tolerance
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.position_tolerance,
    cdr);
  // Member: orientation_tolerance
  cola2_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.orientation_tolerance,
    cdr);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_cola2_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  cola2_msgs::msg::WorldWaypointReq & ros_message)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.header);

  // Member: goal
  cola2_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.goal);

  // Member: altitude_mode
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.altitude_mode = tmp ? true : false;
  }

  // Member: position
  cola2_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.position);

  // Member: altitude
  cdr >> ros_message.altitude;

  // Member: orientation
  cola2_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.orientation);

  // Member: disable_axis
  cola2_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.disable_axis);

  // Member: position_tolerance
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.position_tolerance);

  // Member: orientation_tolerance
  cola2_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.orientation_tolerance);

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_cola2_msgs
get_serialized_size(
  const cola2_msgs::msg::WorldWaypointReq & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: header

  current_alignment +=
    std_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.header, current_alignment);
  // Member: goal

  current_alignment +=
    cola2_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.goal, current_alignment);
  // Member: altitude_mode
  {
    size_t item_size = sizeof(ros_message.altitude_mode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: position

  current_alignment +=
    cola2_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.position, current_alignment);
  // Member: altitude
  {
    size_t item_size = sizeof(ros_message.altitude);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: orientation

  current_alignment +=
    cola2_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.orientation, current_alignment);
  // Member: disable_axis

  current_alignment +=
    cola2_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.disable_axis, current_alignment);
  // Member: position_tolerance

  current_alignment +=
    geometry_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.position_tolerance, current_alignment);
  // Member: orientation_tolerance

  current_alignment +=
    cola2_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.orientation_tolerance, current_alignment);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_cola2_msgs
max_serialized_size_WorldWaypointReq(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: header
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        std_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: goal
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        cola2_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_GoalDescriptor(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: altitude_mode
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: position
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        cola2_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_NED(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: altitude
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: orientation
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        cola2_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_RPY(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: disable_axis
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        cola2_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Bool6Axis(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: position_tolerance
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        geometry_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Vector3(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: orientation_tolerance
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        cola2_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_RPY(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = cola2_msgs::msg::WorldWaypointReq;
    is_plain =
      (
      offsetof(DataType, orientation_tolerance) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _WorldWaypointReq__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const cola2_msgs::msg::WorldWaypointReq *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _WorldWaypointReq__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<cola2_msgs::msg::WorldWaypointReq *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _WorldWaypointReq__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const cola2_msgs::msg::WorldWaypointReq *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _WorldWaypointReq__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_WorldWaypointReq(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _WorldWaypointReq__callbacks = {
  "cola2_msgs::msg",
  "WorldWaypointReq",
  _WorldWaypointReq__cdr_serialize,
  _WorldWaypointReq__cdr_deserialize,
  _WorldWaypointReq__get_serialized_size,
  _WorldWaypointReq__max_serialized_size
};

static rosidl_message_type_support_t _WorldWaypointReq__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_WorldWaypointReq__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace cola2_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_cola2_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<cola2_msgs::msg::WorldWaypointReq>()
{
  return &cola2_msgs::msg::typesupport_fastrtps_cpp::_WorldWaypointReq__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, cola2_msgs, msg, WorldWaypointReq)() {
  return &cola2_msgs::msg::typesupport_fastrtps_cpp::_WorldWaypointReq__handle;
}

#ifdef __cplusplus
}
#endif
