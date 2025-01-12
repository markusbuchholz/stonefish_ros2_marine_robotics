// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from cola2_msgs:action/Pilot.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__ACTION__DETAIL__PILOT__BUILDER_HPP_
#define COLA2_MSGS__ACTION__DETAIL__PILOT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "cola2_msgs/action/detail/pilot__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace cola2_msgs
{

namespace action
{

namespace builder
{

class Init_Pilot_Goal_goal
{
public:
  explicit Init_Pilot_Goal_goal(::cola2_msgs::action::Pilot_Goal & msg)
  : msg_(msg)
  {}
  ::cola2_msgs::action::Pilot_Goal goal(::cola2_msgs::action::Pilot_Goal::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cola2_msgs::action::Pilot_Goal msg_;
};

class Init_Pilot_Goal_controller_type
{
public:
  explicit Init_Pilot_Goal_controller_type(::cola2_msgs::action::Pilot_Goal & msg)
  : msg_(msg)
  {}
  Init_Pilot_Goal_goal controller_type(::cola2_msgs::action::Pilot_Goal::_controller_type_type arg)
  {
    msg_.controller_type = std::move(arg);
    return Init_Pilot_Goal_goal(msg_);
  }

private:
  ::cola2_msgs::action::Pilot_Goal msg_;
};

class Init_Pilot_Goal_timeout
{
public:
  explicit Init_Pilot_Goal_timeout(::cola2_msgs::action::Pilot_Goal & msg)
  : msg_(msg)
  {}
  Init_Pilot_Goal_controller_type timeout(::cola2_msgs::action::Pilot_Goal::_timeout_type arg)
  {
    msg_.timeout = std::move(arg);
    return Init_Pilot_Goal_controller_type(msg_);
  }

private:
  ::cola2_msgs::action::Pilot_Goal msg_;
};

class Init_Pilot_Goal_tolerance_xy
{
public:
  explicit Init_Pilot_Goal_tolerance_xy(::cola2_msgs::action::Pilot_Goal & msg)
  : msg_(msg)
  {}
  Init_Pilot_Goal_timeout tolerance_xy(::cola2_msgs::action::Pilot_Goal::_tolerance_xy_type arg)
  {
    msg_.tolerance_xy = std::move(arg);
    return Init_Pilot_Goal_timeout(msg_);
  }

private:
  ::cola2_msgs::action::Pilot_Goal msg_;
};

class Init_Pilot_Goal_surge_velocity
{
public:
  explicit Init_Pilot_Goal_surge_velocity(::cola2_msgs::action::Pilot_Goal & msg)
  : msg_(msg)
  {}
  Init_Pilot_Goal_tolerance_xy surge_velocity(::cola2_msgs::action::Pilot_Goal::_surge_velocity_type arg)
  {
    msg_.surge_velocity = std::move(arg);
    return Init_Pilot_Goal_tolerance_xy(msg_);
  }

private:
  ::cola2_msgs::action::Pilot_Goal msg_;
};

class Init_Pilot_Goal_heave_mode
{
public:
  explicit Init_Pilot_Goal_heave_mode(::cola2_msgs::action::Pilot_Goal & msg)
  : msg_(msg)
  {}
  Init_Pilot_Goal_surge_velocity heave_mode(::cola2_msgs::action::Pilot_Goal::_heave_mode_type arg)
  {
    msg_.heave_mode = std::move(arg);
    return Init_Pilot_Goal_surge_velocity(msg_);
  }

private:
  ::cola2_msgs::action::Pilot_Goal msg_;
};

class Init_Pilot_Goal_final_altitude
{
public:
  explicit Init_Pilot_Goal_final_altitude(::cola2_msgs::action::Pilot_Goal & msg)
  : msg_(msg)
  {}
  Init_Pilot_Goal_heave_mode final_altitude(::cola2_msgs::action::Pilot_Goal::_final_altitude_type arg)
  {
    msg_.final_altitude = std::move(arg);
    return Init_Pilot_Goal_heave_mode(msg_);
  }

private:
  ::cola2_msgs::action::Pilot_Goal msg_;
};

class Init_Pilot_Goal_final_yaw
{
public:
  explicit Init_Pilot_Goal_final_yaw(::cola2_msgs::action::Pilot_Goal & msg)
  : msg_(msg)
  {}
  Init_Pilot_Goal_final_altitude final_yaw(::cola2_msgs::action::Pilot_Goal::_final_yaw_type arg)
  {
    msg_.final_yaw = std::move(arg);
    return Init_Pilot_Goal_final_altitude(msg_);
  }

private:
  ::cola2_msgs::action::Pilot_Goal msg_;
};

class Init_Pilot_Goal_final_depth
{
public:
  explicit Init_Pilot_Goal_final_depth(::cola2_msgs::action::Pilot_Goal & msg)
  : msg_(msg)
  {}
  Init_Pilot_Goal_final_yaw final_depth(::cola2_msgs::action::Pilot_Goal::_final_depth_type arg)
  {
    msg_.final_depth = std::move(arg);
    return Init_Pilot_Goal_final_yaw(msg_);
  }

private:
  ::cola2_msgs::action::Pilot_Goal msg_;
};

class Init_Pilot_Goal_final_longitude
{
public:
  explicit Init_Pilot_Goal_final_longitude(::cola2_msgs::action::Pilot_Goal & msg)
  : msg_(msg)
  {}
  Init_Pilot_Goal_final_depth final_longitude(::cola2_msgs::action::Pilot_Goal::_final_longitude_type arg)
  {
    msg_.final_longitude = std::move(arg);
    return Init_Pilot_Goal_final_depth(msg_);
  }

private:
  ::cola2_msgs::action::Pilot_Goal msg_;
};

class Init_Pilot_Goal_final_latitude
{
public:
  explicit Init_Pilot_Goal_final_latitude(::cola2_msgs::action::Pilot_Goal & msg)
  : msg_(msg)
  {}
  Init_Pilot_Goal_final_longitude final_latitude(::cola2_msgs::action::Pilot_Goal::_final_latitude_type arg)
  {
    msg_.final_latitude = std::move(arg);
    return Init_Pilot_Goal_final_longitude(msg_);
  }

private:
  ::cola2_msgs::action::Pilot_Goal msg_;
};

class Init_Pilot_Goal_initial_depth
{
public:
  explicit Init_Pilot_Goal_initial_depth(::cola2_msgs::action::Pilot_Goal & msg)
  : msg_(msg)
  {}
  Init_Pilot_Goal_final_latitude initial_depth(::cola2_msgs::action::Pilot_Goal::_initial_depth_type arg)
  {
    msg_.initial_depth = std::move(arg);
    return Init_Pilot_Goal_final_latitude(msg_);
  }

private:
  ::cola2_msgs::action::Pilot_Goal msg_;
};

class Init_Pilot_Goal_initial_longitude
{
public:
  explicit Init_Pilot_Goal_initial_longitude(::cola2_msgs::action::Pilot_Goal & msg)
  : msg_(msg)
  {}
  Init_Pilot_Goal_initial_depth initial_longitude(::cola2_msgs::action::Pilot_Goal::_initial_longitude_type arg)
  {
    msg_.initial_longitude = std::move(arg);
    return Init_Pilot_Goal_initial_depth(msg_);
  }

private:
  ::cola2_msgs::action::Pilot_Goal msg_;
};

class Init_Pilot_Goal_initial_latitude
{
public:
  Init_Pilot_Goal_initial_latitude()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Pilot_Goal_initial_longitude initial_latitude(::cola2_msgs::action::Pilot_Goal::_initial_latitude_type arg)
  {
    msg_.initial_latitude = std::move(arg);
    return Init_Pilot_Goal_initial_longitude(msg_);
  }

private:
  ::cola2_msgs::action::Pilot_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::cola2_msgs::action::Pilot_Goal>()
{
  return cola2_msgs::action::builder::Init_Pilot_Goal_initial_latitude();
}

}  // namespace cola2_msgs


namespace cola2_msgs
{

namespace action
{

namespace builder
{

class Init_Pilot_Result_state
{
public:
  Init_Pilot_Result_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::cola2_msgs::action::Pilot_Result state(::cola2_msgs::action::Pilot_Result::_state_type arg)
  {
    msg_.state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cola2_msgs::action::Pilot_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::cola2_msgs::action::Pilot_Result>()
{
  return cola2_msgs::action::builder::Init_Pilot_Result_state();
}

}  // namespace cola2_msgs


namespace cola2_msgs
{

namespace action
{

namespace builder
{

class Init_Pilot_Feedback_cross_track_error
{
public:
  explicit Init_Pilot_Feedback_cross_track_error(::cola2_msgs::action::Pilot_Feedback & msg)
  : msg_(msg)
  {}
  ::cola2_msgs::action::Pilot_Feedback cross_track_error(::cola2_msgs::action::Pilot_Feedback::_cross_track_error_type arg)
  {
    msg_.cross_track_error = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cola2_msgs::action::Pilot_Feedback msg_;
};

class Init_Pilot_Feedback_distance_to_end
{
public:
  Init_Pilot_Feedback_distance_to_end()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Pilot_Feedback_cross_track_error distance_to_end(::cola2_msgs::action::Pilot_Feedback::_distance_to_end_type arg)
  {
    msg_.distance_to_end = std::move(arg);
    return Init_Pilot_Feedback_cross_track_error(msg_);
  }

private:
  ::cola2_msgs::action::Pilot_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::cola2_msgs::action::Pilot_Feedback>()
{
  return cola2_msgs::action::builder::Init_Pilot_Feedback_distance_to_end();
}

}  // namespace cola2_msgs


namespace cola2_msgs
{

namespace action
{

namespace builder
{

class Init_Pilot_SendGoal_Request_goal
{
public:
  explicit Init_Pilot_SendGoal_Request_goal(::cola2_msgs::action::Pilot_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::cola2_msgs::action::Pilot_SendGoal_Request goal(::cola2_msgs::action::Pilot_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cola2_msgs::action::Pilot_SendGoal_Request msg_;
};

class Init_Pilot_SendGoal_Request_goal_id
{
public:
  Init_Pilot_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Pilot_SendGoal_Request_goal goal_id(::cola2_msgs::action::Pilot_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Pilot_SendGoal_Request_goal(msg_);
  }

private:
  ::cola2_msgs::action::Pilot_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::cola2_msgs::action::Pilot_SendGoal_Request>()
{
  return cola2_msgs::action::builder::Init_Pilot_SendGoal_Request_goal_id();
}

}  // namespace cola2_msgs


namespace cola2_msgs
{

namespace action
{

namespace builder
{

class Init_Pilot_SendGoal_Response_stamp
{
public:
  explicit Init_Pilot_SendGoal_Response_stamp(::cola2_msgs::action::Pilot_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::cola2_msgs::action::Pilot_SendGoal_Response stamp(::cola2_msgs::action::Pilot_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cola2_msgs::action::Pilot_SendGoal_Response msg_;
};

class Init_Pilot_SendGoal_Response_accepted
{
public:
  Init_Pilot_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Pilot_SendGoal_Response_stamp accepted(::cola2_msgs::action::Pilot_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_Pilot_SendGoal_Response_stamp(msg_);
  }

private:
  ::cola2_msgs::action::Pilot_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::cola2_msgs::action::Pilot_SendGoal_Response>()
{
  return cola2_msgs::action::builder::Init_Pilot_SendGoal_Response_accepted();
}

}  // namespace cola2_msgs


namespace cola2_msgs
{

namespace action
{

namespace builder
{

class Init_Pilot_GetResult_Request_goal_id
{
public:
  Init_Pilot_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::cola2_msgs::action::Pilot_GetResult_Request goal_id(::cola2_msgs::action::Pilot_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cola2_msgs::action::Pilot_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::cola2_msgs::action::Pilot_GetResult_Request>()
{
  return cola2_msgs::action::builder::Init_Pilot_GetResult_Request_goal_id();
}

}  // namespace cola2_msgs


namespace cola2_msgs
{

namespace action
{

namespace builder
{

class Init_Pilot_GetResult_Response_result
{
public:
  explicit Init_Pilot_GetResult_Response_result(::cola2_msgs::action::Pilot_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::cola2_msgs::action::Pilot_GetResult_Response result(::cola2_msgs::action::Pilot_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cola2_msgs::action::Pilot_GetResult_Response msg_;
};

class Init_Pilot_GetResult_Response_status
{
public:
  Init_Pilot_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Pilot_GetResult_Response_result status(::cola2_msgs::action::Pilot_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Pilot_GetResult_Response_result(msg_);
  }

private:
  ::cola2_msgs::action::Pilot_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::cola2_msgs::action::Pilot_GetResult_Response>()
{
  return cola2_msgs::action::builder::Init_Pilot_GetResult_Response_status();
}

}  // namespace cola2_msgs


namespace cola2_msgs
{

namespace action
{

namespace builder
{

class Init_Pilot_FeedbackMessage_feedback
{
public:
  explicit Init_Pilot_FeedbackMessage_feedback(::cola2_msgs::action::Pilot_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::cola2_msgs::action::Pilot_FeedbackMessage feedback(::cola2_msgs::action::Pilot_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cola2_msgs::action::Pilot_FeedbackMessage msg_;
};

class Init_Pilot_FeedbackMessage_goal_id
{
public:
  Init_Pilot_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Pilot_FeedbackMessage_feedback goal_id(::cola2_msgs::action::Pilot_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Pilot_FeedbackMessage_feedback(msg_);
  }

private:
  ::cola2_msgs::action::Pilot_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::cola2_msgs::action::Pilot_FeedbackMessage>()
{
  return cola2_msgs::action::builder::Init_Pilot_FeedbackMessage_goal_id();
}

}  // namespace cola2_msgs

#endif  // COLA2_MSGS__ACTION__DETAIL__PILOT__BUILDER_HPP_
