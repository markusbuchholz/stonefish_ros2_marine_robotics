// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from cola2_msgs:msg/Map.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "cola2_msgs/msg/detail/map__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace cola2_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void Map_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) cola2_msgs::msg::Map(_init);
}

void Map_fini_function(void * message_memory)
{
  auto typed_message = static_cast<cola2_msgs::msg::Map *>(message_memory);
  typed_message->~Map();
}

size_t size_function__Map__landmark(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<cola2_msgs::msg::Landmark> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Map__landmark(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<cola2_msgs::msg::Landmark> *>(untyped_member);
  return &member[index];
}

void * get_function__Map__landmark(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<cola2_msgs::msg::Landmark> *>(untyped_member);
  return &member[index];
}

void fetch_function__Map__landmark(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const cola2_msgs::msg::Landmark *>(
    get_const_function__Map__landmark(untyped_member, index));
  auto & value = *reinterpret_cast<cola2_msgs::msg::Landmark *>(untyped_value);
  value = item;
}

void assign_function__Map__landmark(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<cola2_msgs::msg::Landmark *>(
    get_function__Map__landmark(untyped_member, index));
  const auto & value = *reinterpret_cast<const cola2_msgs::msg::Landmark *>(untyped_value);
  item = value;
}

void resize_function__Map__landmark(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<cola2_msgs::msg::Landmark> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Map_message_member_array[2] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs::msg::Map, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "landmark",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<cola2_msgs::msg::Landmark>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cola2_msgs::msg::Map, landmark),  // bytes offset in struct
    nullptr,  // default value
    size_function__Map__landmark,  // size() function pointer
    get_const_function__Map__landmark,  // get_const(index) function pointer
    get_function__Map__landmark,  // get(index) function pointer
    fetch_function__Map__landmark,  // fetch(index, &value) function pointer
    assign_function__Map__landmark,  // assign(index, value) function pointer
    resize_function__Map__landmark  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Map_message_members = {
  "cola2_msgs::msg",  // message namespace
  "Map",  // message name
  2,  // number of fields
  sizeof(cola2_msgs::msg::Map),
  Map_message_member_array,  // message members
  Map_init_function,  // function to initialize message memory (memory has to be allocated)
  Map_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Map_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Map_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace cola2_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<cola2_msgs::msg::Map>()
{
  return &::cola2_msgs::msg::rosidl_typesupport_introspection_cpp::Map_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, cola2_msgs, msg, Map)() {
  return &::cola2_msgs::msg::rosidl_typesupport_introspection_cpp::Map_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
