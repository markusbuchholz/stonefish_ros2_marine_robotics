// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from cola2_msgs:msg/NavSts.idl
// generated code does not contain a copyright notice
#include "cola2_msgs/msg/detail/nav_sts__rosidl_typesupport_fastrtps_cpp.hpp"
#include "cola2_msgs/msg/detail/nav_sts__struct.hpp"

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
  const cola2_msgs::msg::DecimalLatLon &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  cola2_msgs::msg::DecimalLatLon &);
size_t get_serialized_size(
  const cola2_msgs::msg::DecimalLatLon &,
  size_t current_alignment);
size_t
max_serialized_size_DecimalLatLon(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace cola2_msgs

// functions for cola2_msgs::msg::DecimalLatLon already declared above

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

namespace geometry_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const geometry_msgs::msg::Point &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  geometry_msgs::msg::Point &);
size_t get_serialized_size(
  const geometry_msgs::msg::Point &,
  size_t current_alignment);
size_t
max_serialized_size_Point(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace geometry_msgs

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

// functions for cola2_msgs::msg::RPY already declared above

// functions for cola2_msgs::msg::NED already declared above

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
  const cola2_msgs::msg::NavSts & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.header,
    cdr);
  // Member: global_position
  cola2_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.global_position,
    cdr);
  // Member: origin
  cola2_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.origin,
    cdr);
  // Member: position
  cola2_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.position,
    cdr);
  // Member: altitude
  cdr << ros_message.altitude;
  // Member: body_velocity
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.body_velocity,
    cdr);
  // Member: orientation
  cola2_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.orientation,
    cdr);
  // Member: orientation_rate
  cola2_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.orientation_rate,
    cdr);
  // Member: position_variance
  cola2_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.position_variance,
    cdr);
  // Member: orientation_variance
  cola2_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.orientation_variance,
    cdr);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_cola2_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  cola2_msgs::msg::NavSts & ros_message)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.header);

  // Member: global_position
  cola2_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.global_position);

  // Member: origin
  cola2_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.origin);

  // Member: position
  cola2_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.position);

  // Member: altitude
  cdr >> ros_message.altitude;

  // Member: body_velocity
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.body_velocity);

  // Member: orientation
  cola2_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.orientation);

  // Member: orientation_rate
  cola2_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.orientation_rate);

  // Member: position_variance
  cola2_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.position_variance);

  // Member: orientation_variance
  cola2_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.orientation_variance);

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_cola2_msgs
get_serialized_size(
  const cola2_msgs::msg::NavSts & ros_message,
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
  // Member: global_position

  current_alignment +=
    cola2_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.global_position, current_alignment);
  // Member: origin

  current_alignment +=
    cola2_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.origin, current_alignment);
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
  // Member: body_velocity

  current_alignment +=
    geometry_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.body_velocity, current_alignment);
  // Member: orientation

  current_alignment +=
    cola2_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.orientation, current_alignment);
  // Member: orientation_rate

  current_alignment +=
    cola2_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.orientation_rate, current_alignment);
  // Member: position_variance

  current_alignment +=
    cola2_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.position_variance, current_alignment);
  // Member: orientation_variance

  current_alignment +=
    cola2_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.orientation_variance, current_alignment);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_cola2_msgs
max_serialized_size_NavSts(
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

  // Member: global_position
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        cola2_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_DecimalLatLon(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: origin
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        cola2_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_DecimalLatLon(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
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

  // Member: body_velocity
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        geometry_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Point(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
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

  // Member: orientation_rate
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

  // Member: position_variance
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

  // Member: orientation_variance
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
    using DataType = cola2_msgs::msg::NavSts;
    is_plain =
      (
      offsetof(DataType, orientation_variance) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _NavSts__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const cola2_msgs::msg::NavSts *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _NavSts__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<cola2_msgs::msg::NavSts *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _NavSts__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const cola2_msgs::msg::NavSts *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _NavSts__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_NavSts(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _NavSts__callbacks = {
  "cola2_msgs::msg",
  "NavSts",
  _NavSts__cdr_serialize,
  _NavSts__cdr_deserialize,
  _NavSts__get_serialized_size,
  _NavSts__max_serialized_size
};

static rosidl_message_type_support_t _NavSts__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_NavSts__callbacks,
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
get_message_type_support_handle<cola2_msgs::msg::NavSts>()
{
  return &cola2_msgs::msg::typesupport_fastrtps_cpp::_NavSts__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, cola2_msgs, msg, NavSts)() {
  return &cola2_msgs::msg::typesupport_fastrtps_cpp::_NavSts__handle;
}

#ifdef __cplusplus
}
#endif
