// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from cola2_msgs:msg/Bool6Axis.idl
// generated code does not contain a copyright notice
#include "cola2_msgs/msg/detail/bool6_axis__rosidl_typesupport_fastrtps_cpp.hpp"
#include "cola2_msgs/msg/detail/bool6_axis__struct.hpp"

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

namespace cola2_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_cola2_msgs
cdr_serialize(
  const cola2_msgs::msg::Bool6Axis & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: x
  cdr << (ros_message.x ? true : false);
  // Member: y
  cdr << (ros_message.y ? true : false);
  // Member: z
  cdr << (ros_message.z ? true : false);
  // Member: roll
  cdr << (ros_message.roll ? true : false);
  // Member: pitch
  cdr << (ros_message.pitch ? true : false);
  // Member: yaw
  cdr << (ros_message.yaw ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_cola2_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  cola2_msgs::msg::Bool6Axis & ros_message)
{
  // Member: x
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.x = tmp ? true : false;
  }

  // Member: y
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.y = tmp ? true : false;
  }

  // Member: z
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.z = tmp ? true : false;
  }

  // Member: roll
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.roll = tmp ? true : false;
  }

  // Member: pitch
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.pitch = tmp ? true : false;
  }

  // Member: yaw
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.yaw = tmp ? true : false;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_cola2_msgs
get_serialized_size(
  const cola2_msgs::msg::Bool6Axis & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: x
  {
    size_t item_size = sizeof(ros_message.x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: y
  {
    size_t item_size = sizeof(ros_message.y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: z
  {
    size_t item_size = sizeof(ros_message.z);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: roll
  {
    size_t item_size = sizeof(ros_message.roll);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: pitch
  {
    size_t item_size = sizeof(ros_message.pitch);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: yaw
  {
    size_t item_size = sizeof(ros_message.yaw);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_cola2_msgs
max_serialized_size_Bool6Axis(
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


  // Member: x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: z
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: roll
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: pitch
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: yaw
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = cola2_msgs::msg::Bool6Axis;
    is_plain =
      (
      offsetof(DataType, yaw) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _Bool6Axis__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const cola2_msgs::msg::Bool6Axis *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Bool6Axis__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<cola2_msgs::msg::Bool6Axis *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Bool6Axis__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const cola2_msgs::msg::Bool6Axis *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Bool6Axis__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_Bool6Axis(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _Bool6Axis__callbacks = {
  "cola2_msgs::msg",
  "Bool6Axis",
  _Bool6Axis__cdr_serialize,
  _Bool6Axis__cdr_deserialize,
  _Bool6Axis__get_serialized_size,
  _Bool6Axis__max_serialized_size
};

static rosidl_message_type_support_t _Bool6Axis__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Bool6Axis__callbacks,
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
get_message_type_support_handle<cola2_msgs::msg::Bool6Axis>()
{
  return &cola2_msgs::msg::typesupport_fastrtps_cpp::_Bool6Axis__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, cola2_msgs, msg, Bool6Axis)() {
  return &cola2_msgs::msg::typesupport_fastrtps_cpp::_Bool6Axis__handle;
}

#ifdef __cplusplus
}
#endif
