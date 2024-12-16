// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from cola2_msgs:msg/WorldWaypointReq.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__WORLD_WAYPOINT_REQ__BUILDER_HPP_
#define COLA2_MSGS__MSG__DETAIL__WORLD_WAYPOINT_REQ__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "cola2_msgs/msg/detail/world_waypoint_req__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace cola2_msgs
{

namespace msg
{

namespace builder
{

class Init_WorldWaypointReq_orientation_tolerance
{
public:
  explicit Init_WorldWaypointReq_orientation_tolerance(::cola2_msgs::msg::WorldWaypointReq & msg)
  : msg_(msg)
  {}
  ::cola2_msgs::msg::WorldWaypointReq orientation_tolerance(::cola2_msgs::msg::WorldWaypointReq::_orientation_tolerance_type arg)
  {
    msg_.orientation_tolerance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cola2_msgs::msg::WorldWaypointReq msg_;
};

class Init_WorldWaypointReq_position_tolerance
{
public:
  explicit Init_WorldWaypointReq_position_tolerance(::cola2_msgs::msg::WorldWaypointReq & msg)
  : msg_(msg)
  {}
  Init_WorldWaypointReq_orientation_tolerance position_tolerance(::cola2_msgs::msg::WorldWaypointReq::_position_tolerance_type arg)
  {
    msg_.position_tolerance = std::move(arg);
    return Init_WorldWaypointReq_orientation_tolerance(msg_);
  }

private:
  ::cola2_msgs::msg::WorldWaypointReq msg_;
};

class Init_WorldWaypointReq_disable_axis
{
public:
  explicit Init_WorldWaypointReq_disable_axis(::cola2_msgs::msg::WorldWaypointReq & msg)
  : msg_(msg)
  {}
  Init_WorldWaypointReq_position_tolerance disable_axis(::cola2_msgs::msg::WorldWaypointReq::_disable_axis_type arg)
  {
    msg_.disable_axis = std::move(arg);
    return Init_WorldWaypointReq_position_tolerance(msg_);
  }

private:
  ::cola2_msgs::msg::WorldWaypointReq msg_;
};

class Init_WorldWaypointReq_orientation
{
public:
  explicit Init_WorldWaypointReq_orientation(::cola2_msgs::msg::WorldWaypointReq & msg)
  : msg_(msg)
  {}
  Init_WorldWaypointReq_disable_axis orientation(::cola2_msgs::msg::WorldWaypointReq::_orientation_type arg)
  {
    msg_.orientation = std::move(arg);
    return Init_WorldWaypointReq_disable_axis(msg_);
  }

private:
  ::cola2_msgs::msg::WorldWaypointReq msg_;
};

class Init_WorldWaypointReq_altitude
{
public:
  explicit Init_WorldWaypointReq_altitude(::cola2_msgs::msg::WorldWaypointReq & msg)
  : msg_(msg)
  {}
  Init_WorldWaypointReq_orientation altitude(::cola2_msgs::msg::WorldWaypointReq::_altitude_type arg)
  {
    msg_.altitude = std::move(arg);
    return Init_WorldWaypointReq_orientation(msg_);
  }

private:
  ::cola2_msgs::msg::WorldWaypointReq msg_;
};

class Init_WorldWaypointReq_position
{
public:
  explicit Init_WorldWaypointReq_position(::cola2_msgs::msg::WorldWaypointReq & msg)
  : msg_(msg)
  {}
  Init_WorldWaypointReq_altitude position(::cola2_msgs::msg::WorldWaypointReq::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_WorldWaypointReq_altitude(msg_);
  }

private:
  ::cola2_msgs::msg::WorldWaypointReq msg_;
};

class Init_WorldWaypointReq_altitude_mode
{
public:
  explicit Init_WorldWaypointReq_altitude_mode(::cola2_msgs::msg::WorldWaypointReq & msg)
  : msg_(msg)
  {}
  Init_WorldWaypointReq_position altitude_mode(::cola2_msgs::msg::WorldWaypointReq::_altitude_mode_type arg)
  {
    msg_.altitude_mode = std::move(arg);
    return Init_WorldWaypointReq_position(msg_);
  }

private:
  ::cola2_msgs::msg::WorldWaypointReq msg_;
};

class Init_WorldWaypointReq_goal
{
public:
  explicit Init_WorldWaypointReq_goal(::cola2_msgs::msg::WorldWaypointReq & msg)
  : msg_(msg)
  {}
  Init_WorldWaypointReq_altitude_mode goal(::cola2_msgs::msg::WorldWaypointReq::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return Init_WorldWaypointReq_altitude_mode(msg_);
  }

private:
  ::cola2_msgs::msg::WorldWaypointReq msg_;
};

class Init_WorldWaypointReq_header
{
public:
  Init_WorldWaypointReq_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WorldWaypointReq_goal header(::cola2_msgs::msg::WorldWaypointReq::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_WorldWaypointReq_goal(msg_);
  }

private:
  ::cola2_msgs::msg::WorldWaypointReq msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::cola2_msgs::msg::WorldWaypointReq>()
{
  return cola2_msgs::msg::builder::Init_WorldWaypointReq_header();
}

}  // namespace cola2_msgs

#endif  // COLA2_MSGS__MSG__DETAIL__WORLD_WAYPOINT_REQ__BUILDER_HPP_
