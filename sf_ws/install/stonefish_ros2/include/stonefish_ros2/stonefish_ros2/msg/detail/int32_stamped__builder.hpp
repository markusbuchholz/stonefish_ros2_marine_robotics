// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from stonefish_ros2:msg/Int32Stamped.idl
// generated code does not contain a copyright notice

#ifndef STONEFISH_ROS2__MSG__DETAIL__INT32_STAMPED__BUILDER_HPP_
#define STONEFISH_ROS2__MSG__DETAIL__INT32_STAMPED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "stonefish_ros2/msg/detail/int32_stamped__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace stonefish_ros2
{

namespace msg
{

namespace builder
{

class Init_Int32Stamped_data
{
public:
  explicit Init_Int32Stamped_data(::stonefish_ros2::msg::Int32Stamped & msg)
  : msg_(msg)
  {}
  ::stonefish_ros2::msg::Int32Stamped data(::stonefish_ros2::msg::Int32Stamped::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::stonefish_ros2::msg::Int32Stamped msg_;
};

class Init_Int32Stamped_header
{
public:
  Init_Int32Stamped_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Int32Stamped_data header(::stonefish_ros2::msg::Int32Stamped::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Int32Stamped_data(msg_);
  }

private:
  ::stonefish_ros2::msg::Int32Stamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::stonefish_ros2::msg::Int32Stamped>()
{
  return stonefish_ros2::msg::builder::Init_Int32Stamped_header();
}

}  // namespace stonefish_ros2

#endif  // STONEFISH_ROS2__MSG__DETAIL__INT32_STAMPED__BUILDER_HPP_
