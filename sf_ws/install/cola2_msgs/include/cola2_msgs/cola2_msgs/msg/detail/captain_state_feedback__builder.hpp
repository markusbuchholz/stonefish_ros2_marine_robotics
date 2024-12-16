// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from cola2_msgs:msg/CaptainStateFeedback.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__CAPTAIN_STATE_FEEDBACK__BUILDER_HPP_
#define COLA2_MSGS__MSG__DETAIL__CAPTAIN_STATE_FEEDBACK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "cola2_msgs/msg/detail/captain_state_feedback__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace cola2_msgs
{

namespace msg
{

namespace builder
{

class Init_CaptainStateFeedback_keyvalues
{
public:
  explicit Init_CaptainStateFeedback_keyvalues(::cola2_msgs::msg::CaptainStateFeedback & msg)
  : msg_(msg)
  {}
  ::cola2_msgs::msg::CaptainStateFeedback keyvalues(::cola2_msgs::msg::CaptainStateFeedback::_keyvalues_type arg)
  {
    msg_.keyvalues = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cola2_msgs::msg::CaptainStateFeedback msg_;
};

class Init_CaptainStateFeedback_id
{
public:
  explicit Init_CaptainStateFeedback_id(::cola2_msgs::msg::CaptainStateFeedback & msg)
  : msg_(msg)
  {}
  Init_CaptainStateFeedback_keyvalues id(::cola2_msgs::msg::CaptainStateFeedback::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_CaptainStateFeedback_keyvalues(msg_);
  }

private:
  ::cola2_msgs::msg::CaptainStateFeedback msg_;
};

class Init_CaptainStateFeedback_name
{
public:
  explicit Init_CaptainStateFeedback_name(::cola2_msgs::msg::CaptainStateFeedback & msg)
  : msg_(msg)
  {}
  Init_CaptainStateFeedback_id name(::cola2_msgs::msg::CaptainStateFeedback::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_CaptainStateFeedback_id(msg_);
  }

private:
  ::cola2_msgs::msg::CaptainStateFeedback msg_;
};

class Init_CaptainStateFeedback_time_to_finish
{
public:
  explicit Init_CaptainStateFeedback_time_to_finish(::cola2_msgs::msg::CaptainStateFeedback & msg)
  : msg_(msg)
  {}
  Init_CaptainStateFeedback_name time_to_finish(::cola2_msgs::msg::CaptainStateFeedback::_time_to_finish_type arg)
  {
    msg_.time_to_finish = std::move(arg);
    return Init_CaptainStateFeedback_name(msg_);
  }

private:
  ::cola2_msgs::msg::CaptainStateFeedback msg_;
};

class Init_CaptainStateFeedback_state
{
public:
  explicit Init_CaptainStateFeedback_state(::cola2_msgs::msg::CaptainStateFeedback & msg)
  : msg_(msg)
  {}
  Init_CaptainStateFeedback_time_to_finish state(::cola2_msgs::msg::CaptainStateFeedback::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_CaptainStateFeedback_time_to_finish(msg_);
  }

private:
  ::cola2_msgs::msg::CaptainStateFeedback msg_;
};

class Init_CaptainStateFeedback_header
{
public:
  Init_CaptainStateFeedback_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CaptainStateFeedback_state header(::cola2_msgs::msg::CaptainStateFeedback::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_CaptainStateFeedback_state(msg_);
  }

private:
  ::cola2_msgs::msg::CaptainStateFeedback msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::cola2_msgs::msg::CaptainStateFeedback>()
{
  return cola2_msgs::msg::builder::Init_CaptainStateFeedback_header();
}

}  // namespace cola2_msgs

#endif  // COLA2_MSGS__MSG__DETAIL__CAPTAIN_STATE_FEEDBACK__BUILDER_HPP_
