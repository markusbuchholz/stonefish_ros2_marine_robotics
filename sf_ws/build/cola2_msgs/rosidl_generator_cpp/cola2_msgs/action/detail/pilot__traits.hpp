// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from cola2_msgs:action/Pilot.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__ACTION__DETAIL__PILOT__TRAITS_HPP_
#define COLA2_MSGS__ACTION__DETAIL__PILOT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "cola2_msgs/action/detail/pilot__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'goal'
#include "cola2_msgs/msg/detail/goal_descriptor__traits.hpp"

namespace cola2_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const Pilot_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: initial_latitude
  {
    out << "initial_latitude: ";
    rosidl_generator_traits::value_to_yaml(msg.initial_latitude, out);
    out << ", ";
  }

  // member: initial_longitude
  {
    out << "initial_longitude: ";
    rosidl_generator_traits::value_to_yaml(msg.initial_longitude, out);
    out << ", ";
  }

  // member: initial_depth
  {
    out << "initial_depth: ";
    rosidl_generator_traits::value_to_yaml(msg.initial_depth, out);
    out << ", ";
  }

  // member: final_latitude
  {
    out << "final_latitude: ";
    rosidl_generator_traits::value_to_yaml(msg.final_latitude, out);
    out << ", ";
  }

  // member: final_longitude
  {
    out << "final_longitude: ";
    rosidl_generator_traits::value_to_yaml(msg.final_longitude, out);
    out << ", ";
  }

  // member: final_depth
  {
    out << "final_depth: ";
    rosidl_generator_traits::value_to_yaml(msg.final_depth, out);
    out << ", ";
  }

  // member: final_yaw
  {
    out << "final_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.final_yaw, out);
    out << ", ";
  }

  // member: final_altitude
  {
    out << "final_altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.final_altitude, out);
    out << ", ";
  }

  // member: heave_mode
  {
    out << "heave_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.heave_mode, out);
    out << ", ";
  }

  // member: surge_velocity
  {
    out << "surge_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.surge_velocity, out);
    out << ", ";
  }

  // member: tolerance_xy
  {
    out << "tolerance_xy: ";
    rosidl_generator_traits::value_to_yaml(msg.tolerance_xy, out);
    out << ", ";
  }

  // member: timeout
  {
    out << "timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.timeout, out);
    out << ", ";
  }

  // member: controller_type
  {
    out << "controller_type: ";
    rosidl_generator_traits::value_to_yaml(msg.controller_type, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Pilot_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: initial_latitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "initial_latitude: ";
    rosidl_generator_traits::value_to_yaml(msg.initial_latitude, out);
    out << "\n";
  }

  // member: initial_longitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "initial_longitude: ";
    rosidl_generator_traits::value_to_yaml(msg.initial_longitude, out);
    out << "\n";
  }

  // member: initial_depth
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "initial_depth: ";
    rosidl_generator_traits::value_to_yaml(msg.initial_depth, out);
    out << "\n";
  }

  // member: final_latitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "final_latitude: ";
    rosidl_generator_traits::value_to_yaml(msg.final_latitude, out);
    out << "\n";
  }

  // member: final_longitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "final_longitude: ";
    rosidl_generator_traits::value_to_yaml(msg.final_longitude, out);
    out << "\n";
  }

  // member: final_depth
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "final_depth: ";
    rosidl_generator_traits::value_to_yaml(msg.final_depth, out);
    out << "\n";
  }

  // member: final_yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "final_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.final_yaw, out);
    out << "\n";
  }

  // member: final_altitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "final_altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.final_altitude, out);
    out << "\n";
  }

  // member: heave_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "heave_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.heave_mode, out);
    out << "\n";
  }

  // member: surge_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "surge_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.surge_velocity, out);
    out << "\n";
  }

  // member: tolerance_xy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tolerance_xy: ";
    rosidl_generator_traits::value_to_yaml(msg.tolerance_xy, out);
    out << "\n";
  }

  // member: timeout
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.timeout, out);
    out << "\n";
  }

  // member: controller_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "controller_type: ";
    rosidl_generator_traits::value_to_yaml(msg.controller_type, out);
    out << "\n";
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Pilot_Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace cola2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use cola2_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const cola2_msgs::action::Pilot_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  cola2_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use cola2_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const cola2_msgs::action::Pilot_Goal & msg)
{
  return cola2_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<cola2_msgs::action::Pilot_Goal>()
{
  return "cola2_msgs::action::Pilot_Goal";
}

template<>
inline const char * name<cola2_msgs::action::Pilot_Goal>()
{
  return "cola2_msgs/action/Pilot_Goal";
}

template<>
struct has_fixed_size<cola2_msgs::action::Pilot_Goal>
  : std::integral_constant<bool, has_fixed_size<cola2_msgs::msg::GoalDescriptor>::value> {};

template<>
struct has_bounded_size<cola2_msgs::action::Pilot_Goal>
  : std::integral_constant<bool, has_bounded_size<cola2_msgs::msg::GoalDescriptor>::value> {};

template<>
struct is_message<cola2_msgs::action::Pilot_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace cola2_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const Pilot_Result & msg,
  std::ostream & out)
{
  out << "{";
  // member: state
  {
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Pilot_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Pilot_Result & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace cola2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use cola2_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const cola2_msgs::action::Pilot_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  cola2_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use cola2_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const cola2_msgs::action::Pilot_Result & msg)
{
  return cola2_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<cola2_msgs::action::Pilot_Result>()
{
  return "cola2_msgs::action::Pilot_Result";
}

template<>
inline const char * name<cola2_msgs::action::Pilot_Result>()
{
  return "cola2_msgs/action/Pilot_Result";
}

template<>
struct has_fixed_size<cola2_msgs::action::Pilot_Result>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<cola2_msgs::action::Pilot_Result>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<cola2_msgs::action::Pilot_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace cola2_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const Pilot_Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: distance_to_end
  {
    out << "distance_to_end: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_to_end, out);
    out << ", ";
  }

  // member: cross_track_error
  {
    out << "cross_track_error: ";
    rosidl_generator_traits::value_to_yaml(msg.cross_track_error, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Pilot_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: distance_to_end
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance_to_end: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_to_end, out);
    out << "\n";
  }

  // member: cross_track_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cross_track_error: ";
    rosidl_generator_traits::value_to_yaml(msg.cross_track_error, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Pilot_Feedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace cola2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use cola2_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const cola2_msgs::action::Pilot_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  cola2_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use cola2_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const cola2_msgs::action::Pilot_Feedback & msg)
{
  return cola2_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<cola2_msgs::action::Pilot_Feedback>()
{
  return "cola2_msgs::action::Pilot_Feedback";
}

template<>
inline const char * name<cola2_msgs::action::Pilot_Feedback>()
{
  return "cola2_msgs/action/Pilot_Feedback";
}

template<>
struct has_fixed_size<cola2_msgs::action::Pilot_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<cola2_msgs::action::Pilot_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<cola2_msgs::action::Pilot_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "cola2_msgs/action/detail/pilot__traits.hpp"

namespace cola2_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const Pilot_SendGoal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Pilot_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Pilot_SendGoal_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace cola2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use cola2_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const cola2_msgs::action::Pilot_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  cola2_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use cola2_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const cola2_msgs::action::Pilot_SendGoal_Request & msg)
{
  return cola2_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<cola2_msgs::action::Pilot_SendGoal_Request>()
{
  return "cola2_msgs::action::Pilot_SendGoal_Request";
}

template<>
inline const char * name<cola2_msgs::action::Pilot_SendGoal_Request>()
{
  return "cola2_msgs/action/Pilot_SendGoal_Request";
}

template<>
struct has_fixed_size<cola2_msgs::action::Pilot_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<cola2_msgs::action::Pilot_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<cola2_msgs::action::Pilot_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<cola2_msgs::action::Pilot_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<cola2_msgs::action::Pilot_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace cola2_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const Pilot_SendGoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Pilot_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Pilot_SendGoal_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace cola2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use cola2_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const cola2_msgs::action::Pilot_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  cola2_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use cola2_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const cola2_msgs::action::Pilot_SendGoal_Response & msg)
{
  return cola2_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<cola2_msgs::action::Pilot_SendGoal_Response>()
{
  return "cola2_msgs::action::Pilot_SendGoal_Response";
}

template<>
inline const char * name<cola2_msgs::action::Pilot_SendGoal_Response>()
{
  return "cola2_msgs/action/Pilot_SendGoal_Response";
}

template<>
struct has_fixed_size<cola2_msgs::action::Pilot_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<cola2_msgs::action::Pilot_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<cola2_msgs::action::Pilot_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<cola2_msgs::action::Pilot_SendGoal>()
{
  return "cola2_msgs::action::Pilot_SendGoal";
}

template<>
inline const char * name<cola2_msgs::action::Pilot_SendGoal>()
{
  return "cola2_msgs/action/Pilot_SendGoal";
}

template<>
struct has_fixed_size<cola2_msgs::action::Pilot_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<cola2_msgs::action::Pilot_SendGoal_Request>::value &&
    has_fixed_size<cola2_msgs::action::Pilot_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<cola2_msgs::action::Pilot_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<cola2_msgs::action::Pilot_SendGoal_Request>::value &&
    has_bounded_size<cola2_msgs::action::Pilot_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<cola2_msgs::action::Pilot_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<cola2_msgs::action::Pilot_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<cola2_msgs::action::Pilot_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace cola2_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const Pilot_GetResult_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Pilot_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Pilot_GetResult_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace cola2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use cola2_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const cola2_msgs::action::Pilot_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  cola2_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use cola2_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const cola2_msgs::action::Pilot_GetResult_Request & msg)
{
  return cola2_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<cola2_msgs::action::Pilot_GetResult_Request>()
{
  return "cola2_msgs::action::Pilot_GetResult_Request";
}

template<>
inline const char * name<cola2_msgs::action::Pilot_GetResult_Request>()
{
  return "cola2_msgs/action/Pilot_GetResult_Request";
}

template<>
struct has_fixed_size<cola2_msgs::action::Pilot_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<cola2_msgs::action::Pilot_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<cola2_msgs::action::Pilot_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "cola2_msgs/action/detail/pilot__traits.hpp"

namespace cola2_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const Pilot_GetResult_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    to_flow_style_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Pilot_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_block_style_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Pilot_GetResult_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace cola2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use cola2_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const cola2_msgs::action::Pilot_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  cola2_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use cola2_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const cola2_msgs::action::Pilot_GetResult_Response & msg)
{
  return cola2_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<cola2_msgs::action::Pilot_GetResult_Response>()
{
  return "cola2_msgs::action::Pilot_GetResult_Response";
}

template<>
inline const char * name<cola2_msgs::action::Pilot_GetResult_Response>()
{
  return "cola2_msgs/action/Pilot_GetResult_Response";
}

template<>
struct has_fixed_size<cola2_msgs::action::Pilot_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<cola2_msgs::action::Pilot_Result>::value> {};

template<>
struct has_bounded_size<cola2_msgs::action::Pilot_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<cola2_msgs::action::Pilot_Result>::value> {};

template<>
struct is_message<cola2_msgs::action::Pilot_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<cola2_msgs::action::Pilot_GetResult>()
{
  return "cola2_msgs::action::Pilot_GetResult";
}

template<>
inline const char * name<cola2_msgs::action::Pilot_GetResult>()
{
  return "cola2_msgs/action/Pilot_GetResult";
}

template<>
struct has_fixed_size<cola2_msgs::action::Pilot_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<cola2_msgs::action::Pilot_GetResult_Request>::value &&
    has_fixed_size<cola2_msgs::action::Pilot_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<cola2_msgs::action::Pilot_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<cola2_msgs::action::Pilot_GetResult_Request>::value &&
    has_bounded_size<cola2_msgs::action::Pilot_GetResult_Response>::value
  >
{
};

template<>
struct is_service<cola2_msgs::action::Pilot_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<cola2_msgs::action::Pilot_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<cola2_msgs::action::Pilot_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "cola2_msgs/action/detail/pilot__traits.hpp"

namespace cola2_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const Pilot_FeedbackMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: feedback
  {
    out << "feedback: ";
    to_flow_style_yaml(msg.feedback, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Pilot_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback:\n";
    to_block_style_yaml(msg.feedback, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Pilot_FeedbackMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace cola2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use cola2_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const cola2_msgs::action::Pilot_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  cola2_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use cola2_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const cola2_msgs::action::Pilot_FeedbackMessage & msg)
{
  return cola2_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<cola2_msgs::action::Pilot_FeedbackMessage>()
{
  return "cola2_msgs::action::Pilot_FeedbackMessage";
}

template<>
inline const char * name<cola2_msgs::action::Pilot_FeedbackMessage>()
{
  return "cola2_msgs/action/Pilot_FeedbackMessage";
}

template<>
struct has_fixed_size<cola2_msgs::action::Pilot_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<cola2_msgs::action::Pilot_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<cola2_msgs::action::Pilot_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<cola2_msgs::action::Pilot_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<cola2_msgs::action::Pilot_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<cola2_msgs::action::Pilot>
  : std::true_type
{
};

template<>
struct is_action_goal<cola2_msgs::action::Pilot_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<cola2_msgs::action::Pilot_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<cola2_msgs::action::Pilot_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // COLA2_MSGS__ACTION__DETAIL__PILOT__TRAITS_HPP_
