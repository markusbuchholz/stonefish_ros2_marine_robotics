// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from cola2_msgs:msg/Landmark.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__LANDMARK__BUILDER_HPP_
#define COLA2_MSGS__MSG__DETAIL__LANDMARK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "cola2_msgs/msg/detail/landmark__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace cola2_msgs
{

namespace msg
{

namespace builder
{

class Init_Landmark_pose
{
public:
  explicit Init_Landmark_pose(::cola2_msgs::msg::Landmark & msg)
  : msg_(msg)
  {}
  ::cola2_msgs::msg::Landmark pose(::cola2_msgs::msg::Landmark::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cola2_msgs::msg::Landmark msg_;
};

class Init_Landmark_landmark_id
{
public:
  explicit Init_Landmark_landmark_id(::cola2_msgs::msg::Landmark & msg)
  : msg_(msg)
  {}
  Init_Landmark_pose landmark_id(::cola2_msgs::msg::Landmark::_landmark_id_type arg)
  {
    msg_.landmark_id = std::move(arg);
    return Init_Landmark_pose(msg_);
  }

private:
  ::cola2_msgs::msg::Landmark msg_;
};

class Init_Landmark_last_update
{
public:
  Init_Landmark_last_update()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Landmark_landmark_id last_update(::cola2_msgs::msg::Landmark::_last_update_type arg)
  {
    msg_.last_update = std::move(arg);
    return Init_Landmark_landmark_id(msg_);
  }

private:
  ::cola2_msgs::msg::Landmark msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::cola2_msgs::msg::Landmark>()
{
  return cola2_msgs::msg::builder::Init_Landmark_last_update();
}

}  // namespace cola2_msgs

#endif  // COLA2_MSGS__MSG__DETAIL__LANDMARK__BUILDER_HPP_
