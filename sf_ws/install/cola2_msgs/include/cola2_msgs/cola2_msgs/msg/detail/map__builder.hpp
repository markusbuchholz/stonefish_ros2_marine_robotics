// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from cola2_msgs:msg/Map.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__MAP__BUILDER_HPP_
#define COLA2_MSGS__MSG__DETAIL__MAP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "cola2_msgs/msg/detail/map__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace cola2_msgs
{

namespace msg
{

namespace builder
{

class Init_Map_landmark
{
public:
  explicit Init_Map_landmark(::cola2_msgs::msg::Map & msg)
  : msg_(msg)
  {}
  ::cola2_msgs::msg::Map landmark(::cola2_msgs::msg::Map::_landmark_type arg)
  {
    msg_.landmark = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cola2_msgs::msg::Map msg_;
};

class Init_Map_header
{
public:
  Init_Map_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Map_landmark header(::cola2_msgs::msg::Map::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Map_landmark(msg_);
  }

private:
  ::cola2_msgs::msg::Map msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::cola2_msgs::msg::Map>()
{
  return cola2_msgs::msg::builder::Init_Map_header();
}

}  // namespace cola2_msgs

#endif  // COLA2_MSGS__MSG__DETAIL__MAP__BUILDER_HPP_
