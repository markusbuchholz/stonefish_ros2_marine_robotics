// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from cola2_msgs:msg/Float32Stamped.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__FLOAT32_STAMPED__BUILDER_HPP_
#define COLA2_MSGS__MSG__DETAIL__FLOAT32_STAMPED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "cola2_msgs/msg/detail/float32_stamped__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace cola2_msgs
{

namespace msg
{

namespace builder
{

class Init_Float32Stamped_data
{
public:
  explicit Init_Float32Stamped_data(::cola2_msgs::msg::Float32Stamped & msg)
  : msg_(msg)
  {}
  ::cola2_msgs::msg::Float32Stamped data(::cola2_msgs::msg::Float32Stamped::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cola2_msgs::msg::Float32Stamped msg_;
};

class Init_Float32Stamped_header
{
public:
  Init_Float32Stamped_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Float32Stamped_data header(::cola2_msgs::msg::Float32Stamped::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Float32Stamped_data(msg_);
  }

private:
  ::cola2_msgs::msg::Float32Stamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::cola2_msgs::msg::Float32Stamped>()
{
  return cola2_msgs::msg::builder::Init_Float32Stamped_header();
}

}  // namespace cola2_msgs

#endif  // COLA2_MSGS__MSG__DETAIL__FLOAT32_STAMPED__BUILDER_HPP_
