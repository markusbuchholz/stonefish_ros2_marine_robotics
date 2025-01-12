// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from cola2_msgs:msg/NavSts.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__NAV_STS__BUILDER_HPP_
#define COLA2_MSGS__MSG__DETAIL__NAV_STS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "cola2_msgs/msg/detail/nav_sts__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace cola2_msgs
{

namespace msg
{

namespace builder
{

class Init_NavSts_orientation_variance
{
public:
  explicit Init_NavSts_orientation_variance(::cola2_msgs::msg::NavSts & msg)
  : msg_(msg)
  {}
  ::cola2_msgs::msg::NavSts orientation_variance(::cola2_msgs::msg::NavSts::_orientation_variance_type arg)
  {
    msg_.orientation_variance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cola2_msgs::msg::NavSts msg_;
};

class Init_NavSts_position_variance
{
public:
  explicit Init_NavSts_position_variance(::cola2_msgs::msg::NavSts & msg)
  : msg_(msg)
  {}
  Init_NavSts_orientation_variance position_variance(::cola2_msgs::msg::NavSts::_position_variance_type arg)
  {
    msg_.position_variance = std::move(arg);
    return Init_NavSts_orientation_variance(msg_);
  }

private:
  ::cola2_msgs::msg::NavSts msg_;
};

class Init_NavSts_orientation_rate
{
public:
  explicit Init_NavSts_orientation_rate(::cola2_msgs::msg::NavSts & msg)
  : msg_(msg)
  {}
  Init_NavSts_position_variance orientation_rate(::cola2_msgs::msg::NavSts::_orientation_rate_type arg)
  {
    msg_.orientation_rate = std::move(arg);
    return Init_NavSts_position_variance(msg_);
  }

private:
  ::cola2_msgs::msg::NavSts msg_;
};

class Init_NavSts_orientation
{
public:
  explicit Init_NavSts_orientation(::cola2_msgs::msg::NavSts & msg)
  : msg_(msg)
  {}
  Init_NavSts_orientation_rate orientation(::cola2_msgs::msg::NavSts::_orientation_type arg)
  {
    msg_.orientation = std::move(arg);
    return Init_NavSts_orientation_rate(msg_);
  }

private:
  ::cola2_msgs::msg::NavSts msg_;
};

class Init_NavSts_body_velocity
{
public:
  explicit Init_NavSts_body_velocity(::cola2_msgs::msg::NavSts & msg)
  : msg_(msg)
  {}
  Init_NavSts_orientation body_velocity(::cola2_msgs::msg::NavSts::_body_velocity_type arg)
  {
    msg_.body_velocity = std::move(arg);
    return Init_NavSts_orientation(msg_);
  }

private:
  ::cola2_msgs::msg::NavSts msg_;
};

class Init_NavSts_altitude
{
public:
  explicit Init_NavSts_altitude(::cola2_msgs::msg::NavSts & msg)
  : msg_(msg)
  {}
  Init_NavSts_body_velocity altitude(::cola2_msgs::msg::NavSts::_altitude_type arg)
  {
    msg_.altitude = std::move(arg);
    return Init_NavSts_body_velocity(msg_);
  }

private:
  ::cola2_msgs::msg::NavSts msg_;
};

class Init_NavSts_position
{
public:
  explicit Init_NavSts_position(::cola2_msgs::msg::NavSts & msg)
  : msg_(msg)
  {}
  Init_NavSts_altitude position(::cola2_msgs::msg::NavSts::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_NavSts_altitude(msg_);
  }

private:
  ::cola2_msgs::msg::NavSts msg_;
};

class Init_NavSts_origin
{
public:
  explicit Init_NavSts_origin(::cola2_msgs::msg::NavSts & msg)
  : msg_(msg)
  {}
  Init_NavSts_position origin(::cola2_msgs::msg::NavSts::_origin_type arg)
  {
    msg_.origin = std::move(arg);
    return Init_NavSts_position(msg_);
  }

private:
  ::cola2_msgs::msg::NavSts msg_;
};

class Init_NavSts_global_position
{
public:
  explicit Init_NavSts_global_position(::cola2_msgs::msg::NavSts & msg)
  : msg_(msg)
  {}
  Init_NavSts_origin global_position(::cola2_msgs::msg::NavSts::_global_position_type arg)
  {
    msg_.global_position = std::move(arg);
    return Init_NavSts_origin(msg_);
  }

private:
  ::cola2_msgs::msg::NavSts msg_;
};

class Init_NavSts_header
{
public:
  Init_NavSts_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavSts_global_position header(::cola2_msgs::msg::NavSts::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_NavSts_global_position(msg_);
  }

private:
  ::cola2_msgs::msg::NavSts msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::cola2_msgs::msg::NavSts>()
{
  return cola2_msgs::msg::builder::Init_NavSts_header();
}

}  // namespace cola2_msgs

#endif  // COLA2_MSGS__MSG__DETAIL__NAV_STS__BUILDER_HPP_
