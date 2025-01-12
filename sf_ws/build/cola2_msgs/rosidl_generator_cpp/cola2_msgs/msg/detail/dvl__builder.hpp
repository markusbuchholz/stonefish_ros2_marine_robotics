// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from cola2_msgs:msg/DVL.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__DVL__BUILDER_HPP_
#define COLA2_MSGS__MSG__DETAIL__DVL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "cola2_msgs/msg/detail/dvl__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace cola2_msgs
{

namespace msg
{

namespace builder
{

class Init_DVL_beams
{
public:
  explicit Init_DVL_beams(::cola2_msgs::msg::DVL & msg)
  : msg_(msg)
  {}
  ::cola2_msgs::msg::DVL beams(::cola2_msgs::msg::DVL::_beams_type arg)
  {
    msg_.beams = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cola2_msgs::msg::DVL msg_;
};

class Init_DVL_altitude
{
public:
  explicit Init_DVL_altitude(::cola2_msgs::msg::DVL & msg)
  : msg_(msg)
  {}
  Init_DVL_beams altitude(::cola2_msgs::msg::DVL::_altitude_type arg)
  {
    msg_.altitude = std::move(arg);
    return Init_DVL_beams(msg_);
  }

private:
  ::cola2_msgs::msg::DVL msg_;
};

class Init_DVL_velocity_covariance
{
public:
  explicit Init_DVL_velocity_covariance(::cola2_msgs::msg::DVL & msg)
  : msg_(msg)
  {}
  Init_DVL_altitude velocity_covariance(::cola2_msgs::msg::DVL::_velocity_covariance_type arg)
  {
    msg_.velocity_covariance = std::move(arg);
    return Init_DVL_altitude(msg_);
  }

private:
  ::cola2_msgs::msg::DVL msg_;
};

class Init_DVL_velocity
{
public:
  explicit Init_DVL_velocity(::cola2_msgs::msg::DVL & msg)
  : msg_(msg)
  {}
  Init_DVL_velocity_covariance velocity(::cola2_msgs::msg::DVL::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_DVL_velocity_covariance(msg_);
  }

private:
  ::cola2_msgs::msg::DVL msg_;
};

class Init_DVL_header
{
public:
  Init_DVL_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DVL_velocity header(::cola2_msgs::msg::DVL::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DVL_velocity(msg_);
  }

private:
  ::cola2_msgs::msg::DVL msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::cola2_msgs::msg::DVL>()
{
  return cola2_msgs::msg::builder::Init_DVL_header();
}

}  // namespace cola2_msgs

#endif  // COLA2_MSGS__MSG__DETAIL__DVL__BUILDER_HPP_
