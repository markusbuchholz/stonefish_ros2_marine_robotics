// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from cola2_msgs:srv/Section.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__SRV__DETAIL__SECTION__BUILDER_HPP_
#define COLA2_MSGS__SRV__DETAIL__SECTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "cola2_msgs/srv/detail/section__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace cola2_msgs
{

namespace srv
{

namespace builder
{

class Init_Section_Request_no_altitude_goes_up
{
public:
  explicit Init_Section_Request_no_altitude_goes_up(::cola2_msgs::srv::Section_Request & msg)
  : msg_(msg)
  {}
  ::cola2_msgs::srv::Section_Request no_altitude_goes_up(::cola2_msgs::srv::Section_Request::_no_altitude_goes_up_type arg)
  {
    msg_.no_altitude_goes_up = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cola2_msgs::srv::Section_Request msg_;
};

class Init_Section_Request_timeout
{
public:
  explicit Init_Section_Request_timeout(::cola2_msgs::srv::Section_Request & msg)
  : msg_(msg)
  {}
  Init_Section_Request_no_altitude_goes_up timeout(::cola2_msgs::srv::Section_Request::_timeout_type arg)
  {
    msg_.timeout = std::move(arg);
    return Init_Section_Request_no_altitude_goes_up(msg_);
  }

private:
  ::cola2_msgs::srv::Section_Request msg_;
};

class Init_Section_Request_tolerance_xy
{
public:
  explicit Init_Section_Request_tolerance_xy(::cola2_msgs::srv::Section_Request & msg)
  : msg_(msg)
  {}
  Init_Section_Request_timeout tolerance_xy(::cola2_msgs::srv::Section_Request::_tolerance_xy_type arg)
  {
    msg_.tolerance_xy = std::move(arg);
    return Init_Section_Request_timeout(msg_);
  }

private:
  ::cola2_msgs::srv::Section_Request msg_;
};

class Init_Section_Request_surge_velocity
{
public:
  explicit Init_Section_Request_surge_velocity(::cola2_msgs::srv::Section_Request & msg)
  : msg_(msg)
  {}
  Init_Section_Request_tolerance_xy surge_velocity(::cola2_msgs::srv::Section_Request::_surge_velocity_type arg)
  {
    msg_.surge_velocity = std::move(arg);
    return Init_Section_Request_tolerance_xy(msg_);
  }

private:
  ::cola2_msgs::srv::Section_Request msg_;
};

class Init_Section_Request_heave_mode
{
public:
  explicit Init_Section_Request_heave_mode(::cola2_msgs::srv::Section_Request & msg)
  : msg_(msg)
  {}
  Init_Section_Request_surge_velocity heave_mode(::cola2_msgs::srv::Section_Request::_heave_mode_type arg)
  {
    msg_.heave_mode = std::move(arg);
    return Init_Section_Request_surge_velocity(msg_);
  }

private:
  ::cola2_msgs::srv::Section_Request msg_;
};

class Init_Section_Request_reference
{
public:
  explicit Init_Section_Request_reference(::cola2_msgs::srv::Section_Request & msg)
  : msg_(msg)
  {}
  Init_Section_Request_heave_mode reference(::cola2_msgs::srv::Section_Request::_reference_type arg)
  {
    msg_.reference = std::move(arg);
    return Init_Section_Request_heave_mode(msg_);
  }

private:
  ::cola2_msgs::srv::Section_Request msg_;
};

class Init_Section_Request_final_altitude
{
public:
  explicit Init_Section_Request_final_altitude(::cola2_msgs::srv::Section_Request & msg)
  : msg_(msg)
  {}
  Init_Section_Request_reference final_altitude(::cola2_msgs::srv::Section_Request::_final_altitude_type arg)
  {
    msg_.final_altitude = std::move(arg);
    return Init_Section_Request_reference(msg_);
  }

private:
  ::cola2_msgs::srv::Section_Request msg_;
};

class Init_Section_Request_final_depth
{
public:
  explicit Init_Section_Request_final_depth(::cola2_msgs::srv::Section_Request & msg)
  : msg_(msg)
  {}
  Init_Section_Request_final_altitude final_depth(::cola2_msgs::srv::Section_Request::_final_depth_type arg)
  {
    msg_.final_depth = std::move(arg);
    return Init_Section_Request_final_altitude(msg_);
  }

private:
  ::cola2_msgs::srv::Section_Request msg_;
};

class Init_Section_Request_final_y
{
public:
  explicit Init_Section_Request_final_y(::cola2_msgs::srv::Section_Request & msg)
  : msg_(msg)
  {}
  Init_Section_Request_final_depth final_y(::cola2_msgs::srv::Section_Request::_final_y_type arg)
  {
    msg_.final_y = std::move(arg);
    return Init_Section_Request_final_depth(msg_);
  }

private:
  ::cola2_msgs::srv::Section_Request msg_;
};

class Init_Section_Request_final_x
{
public:
  explicit Init_Section_Request_final_x(::cola2_msgs::srv::Section_Request & msg)
  : msg_(msg)
  {}
  Init_Section_Request_final_y final_x(::cola2_msgs::srv::Section_Request::_final_x_type arg)
  {
    msg_.final_x = std::move(arg);
    return Init_Section_Request_final_y(msg_);
  }

private:
  ::cola2_msgs::srv::Section_Request msg_;
};

class Init_Section_Request_initial_depth
{
public:
  explicit Init_Section_Request_initial_depth(::cola2_msgs::srv::Section_Request & msg)
  : msg_(msg)
  {}
  Init_Section_Request_final_x initial_depth(::cola2_msgs::srv::Section_Request::_initial_depth_type arg)
  {
    msg_.initial_depth = std::move(arg);
    return Init_Section_Request_final_x(msg_);
  }

private:
  ::cola2_msgs::srv::Section_Request msg_;
};

class Init_Section_Request_initial_y
{
public:
  explicit Init_Section_Request_initial_y(::cola2_msgs::srv::Section_Request & msg)
  : msg_(msg)
  {}
  Init_Section_Request_initial_depth initial_y(::cola2_msgs::srv::Section_Request::_initial_y_type arg)
  {
    msg_.initial_y = std::move(arg);
    return Init_Section_Request_initial_depth(msg_);
  }

private:
  ::cola2_msgs::srv::Section_Request msg_;
};

class Init_Section_Request_initial_x
{
public:
  Init_Section_Request_initial_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Section_Request_initial_y initial_x(::cola2_msgs::srv::Section_Request::_initial_x_type arg)
  {
    msg_.initial_x = std::move(arg);
    return Init_Section_Request_initial_y(msg_);
  }

private:
  ::cola2_msgs::srv::Section_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::cola2_msgs::srv::Section_Request>()
{
  return cola2_msgs::srv::builder::Init_Section_Request_initial_x();
}

}  // namespace cola2_msgs


namespace cola2_msgs
{

namespace srv
{

namespace builder
{

class Init_Section_Response_message
{
public:
  explicit Init_Section_Response_message(::cola2_msgs::srv::Section_Response & msg)
  : msg_(msg)
  {}
  ::cola2_msgs::srv::Section_Response message(::cola2_msgs::srv::Section_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cola2_msgs::srv::Section_Response msg_;
};

class Init_Section_Response_success
{
public:
  Init_Section_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Section_Response_message success(::cola2_msgs::srv::Section_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_Section_Response_message(msg_);
  }

private:
  ::cola2_msgs::srv::Section_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::cola2_msgs::srv::Section_Response>()
{
  return cola2_msgs::srv::builder::Init_Section_Response_success();
}

}  // namespace cola2_msgs

#endif  // COLA2_MSGS__SRV__DETAIL__SECTION__BUILDER_HPP_
