// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from cola2_msgs:action/Pilot.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__ACTION__DETAIL__PILOT__STRUCT_HPP_
#define COLA2_MSGS__ACTION__DETAIL__PILOT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'goal'
#include "cola2_msgs/msg/detail/goal_descriptor__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__cola2_msgs__action__Pilot_Goal __attribute__((deprecated))
#else
# define DEPRECATED__cola2_msgs__action__Pilot_Goal __declspec(deprecated)
#endif

namespace cola2_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Pilot_Goal_
{
  using Type = Pilot_Goal_<ContainerAllocator>;

  explicit Pilot_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->initial_latitude = 0.0;
      this->initial_longitude = 0.0;
      this->initial_depth = 0.0;
      this->final_latitude = 0.0;
      this->final_longitude = 0.0;
      this->final_depth = 0.0;
      this->final_yaw = 0.0;
      this->final_altitude = 0.0;
      this->heave_mode = 0ull;
      this->surge_velocity = 0.0;
      this->tolerance_xy = 0.0;
      this->timeout = 0.0;
      this->controller_type = 0ull;
    }
  }

  explicit Pilot_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->initial_latitude = 0.0;
      this->initial_longitude = 0.0;
      this->initial_depth = 0.0;
      this->final_latitude = 0.0;
      this->final_longitude = 0.0;
      this->final_depth = 0.0;
      this->final_yaw = 0.0;
      this->final_altitude = 0.0;
      this->heave_mode = 0ull;
      this->surge_velocity = 0.0;
      this->tolerance_xy = 0.0;
      this->timeout = 0.0;
      this->controller_type = 0ull;
    }
  }

  // field types and members
  using _initial_latitude_type =
    double;
  _initial_latitude_type initial_latitude;
  using _initial_longitude_type =
    double;
  _initial_longitude_type initial_longitude;
  using _initial_depth_type =
    double;
  _initial_depth_type initial_depth;
  using _final_latitude_type =
    double;
  _final_latitude_type final_latitude;
  using _final_longitude_type =
    double;
  _final_longitude_type final_longitude;
  using _final_depth_type =
    double;
  _final_depth_type final_depth;
  using _final_yaw_type =
    double;
  _final_yaw_type final_yaw;
  using _final_altitude_type =
    double;
  _final_altitude_type final_altitude;
  using _heave_mode_type =
    uint64_t;
  _heave_mode_type heave_mode;
  using _surge_velocity_type =
    double;
  _surge_velocity_type surge_velocity;
  using _tolerance_xy_type =
    double;
  _tolerance_xy_type tolerance_xy;
  using _timeout_type =
    double;
  _timeout_type timeout;
  using _controller_type_type =
    uint64_t;
  _controller_type_type controller_type;
  using _goal_type =
    cola2_msgs::msg::GoalDescriptor_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__initial_latitude(
    const double & _arg)
  {
    this->initial_latitude = _arg;
    return *this;
  }
  Type & set__initial_longitude(
    const double & _arg)
  {
    this->initial_longitude = _arg;
    return *this;
  }
  Type & set__initial_depth(
    const double & _arg)
  {
    this->initial_depth = _arg;
    return *this;
  }
  Type & set__final_latitude(
    const double & _arg)
  {
    this->final_latitude = _arg;
    return *this;
  }
  Type & set__final_longitude(
    const double & _arg)
  {
    this->final_longitude = _arg;
    return *this;
  }
  Type & set__final_depth(
    const double & _arg)
  {
    this->final_depth = _arg;
    return *this;
  }
  Type & set__final_yaw(
    const double & _arg)
  {
    this->final_yaw = _arg;
    return *this;
  }
  Type & set__final_altitude(
    const double & _arg)
  {
    this->final_altitude = _arg;
    return *this;
  }
  Type & set__heave_mode(
    const uint64_t & _arg)
  {
    this->heave_mode = _arg;
    return *this;
  }
  Type & set__surge_velocity(
    const double & _arg)
  {
    this->surge_velocity = _arg;
    return *this;
  }
  Type & set__tolerance_xy(
    const double & _arg)
  {
    this->tolerance_xy = _arg;
    return *this;
  }
  Type & set__timeout(
    const double & _arg)
  {
    this->timeout = _arg;
    return *this;
  }
  Type & set__controller_type(
    const uint64_t & _arg)
  {
    this->controller_type = _arg;
    return *this;
  }
  Type & set__goal(
    const cola2_msgs::msg::GoalDescriptor_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint64_t DEPTH =
    0u;
  static constexpr uint64_t ALTITUDE =
    1u;
  static constexpr uint64_t BOTH =
    2u;
  static constexpr uint64_t SECTION =
    0u;
  static constexpr uint64_t ANCHOR =
    1u;
  static constexpr uint64_t HOLONOMIC_KEEP_POSITION =
    2u;

  // pointer types
  using RawPtr =
    cola2_msgs::action::Pilot_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const cola2_msgs::action::Pilot_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cola2_msgs::action::Pilot_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cola2_msgs::action::Pilot_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::action::Pilot_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::action::Pilot_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::action::Pilot_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::action::Pilot_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cola2_msgs::action::Pilot_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cola2_msgs::action::Pilot_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cola2_msgs__action__Pilot_Goal
    std::shared_ptr<cola2_msgs::action::Pilot_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cola2_msgs__action__Pilot_Goal
    std::shared_ptr<cola2_msgs::action::Pilot_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Pilot_Goal_ & other) const
  {
    if (this->initial_latitude != other.initial_latitude) {
      return false;
    }
    if (this->initial_longitude != other.initial_longitude) {
      return false;
    }
    if (this->initial_depth != other.initial_depth) {
      return false;
    }
    if (this->final_latitude != other.final_latitude) {
      return false;
    }
    if (this->final_longitude != other.final_longitude) {
      return false;
    }
    if (this->final_depth != other.final_depth) {
      return false;
    }
    if (this->final_yaw != other.final_yaw) {
      return false;
    }
    if (this->final_altitude != other.final_altitude) {
      return false;
    }
    if (this->heave_mode != other.heave_mode) {
      return false;
    }
    if (this->surge_velocity != other.surge_velocity) {
      return false;
    }
    if (this->tolerance_xy != other.tolerance_xy) {
      return false;
    }
    if (this->timeout != other.timeout) {
      return false;
    }
    if (this->controller_type != other.controller_type) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const Pilot_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Pilot_Goal_

// alias to use template instance with default allocator
using Pilot_Goal =
  cola2_msgs::action::Pilot_Goal_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint64_t Pilot_Goal_<ContainerAllocator>::DEPTH;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint64_t Pilot_Goal_<ContainerAllocator>::ALTITUDE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint64_t Pilot_Goal_<ContainerAllocator>::BOTH;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint64_t Pilot_Goal_<ContainerAllocator>::SECTION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint64_t Pilot_Goal_<ContainerAllocator>::ANCHOR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint64_t Pilot_Goal_<ContainerAllocator>::HOLONOMIC_KEEP_POSITION;
#endif  // __cplusplus < 201703L

}  // namespace action

}  // namespace cola2_msgs


#ifndef _WIN32
# define DEPRECATED__cola2_msgs__action__Pilot_Result __attribute__((deprecated))
#else
# define DEPRECATED__cola2_msgs__action__Pilot_Result __declspec(deprecated)
#endif

namespace cola2_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Pilot_Result_
{
  using Type = Pilot_Result_<ContainerAllocator>;

  explicit Pilot_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0ull;
    }
  }

  explicit Pilot_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0ull;
    }
  }

  // field types and members
  using _state_type =
    uint64_t;
  _state_type state;

  // setters for named parameter idiom
  Type & set__state(
    const uint64_t & _arg)
  {
    this->state = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint64_t SUCCESS =
    0u;
  static constexpr uint64_t TIMEOUT =
    1u;
  static constexpr uint64_t FAILURE =
    2u;

  // pointer types
  using RawPtr =
    cola2_msgs::action::Pilot_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const cola2_msgs::action::Pilot_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cola2_msgs::action::Pilot_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cola2_msgs::action::Pilot_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::action::Pilot_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::action::Pilot_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::action::Pilot_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::action::Pilot_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cola2_msgs::action::Pilot_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cola2_msgs::action::Pilot_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cola2_msgs__action__Pilot_Result
    std::shared_ptr<cola2_msgs::action::Pilot_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cola2_msgs__action__Pilot_Result
    std::shared_ptr<cola2_msgs::action::Pilot_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Pilot_Result_ & other) const
  {
    if (this->state != other.state) {
      return false;
    }
    return true;
  }
  bool operator!=(const Pilot_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Pilot_Result_

// alias to use template instance with default allocator
using Pilot_Result =
  cola2_msgs::action::Pilot_Result_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint64_t Pilot_Result_<ContainerAllocator>::SUCCESS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint64_t Pilot_Result_<ContainerAllocator>::TIMEOUT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint64_t Pilot_Result_<ContainerAllocator>::FAILURE;
#endif  // __cplusplus < 201703L

}  // namespace action

}  // namespace cola2_msgs


#ifndef _WIN32
# define DEPRECATED__cola2_msgs__action__Pilot_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__cola2_msgs__action__Pilot_Feedback __declspec(deprecated)
#endif

namespace cola2_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Pilot_Feedback_
{
  using Type = Pilot_Feedback_<ContainerAllocator>;

  explicit Pilot_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->distance_to_end = 0.0;
      this->cross_track_error = 0.0;
    }
  }

  explicit Pilot_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->distance_to_end = 0.0;
      this->cross_track_error = 0.0;
    }
  }

  // field types and members
  using _distance_to_end_type =
    double;
  _distance_to_end_type distance_to_end;
  using _cross_track_error_type =
    double;
  _cross_track_error_type cross_track_error;

  // setters for named parameter idiom
  Type & set__distance_to_end(
    const double & _arg)
  {
    this->distance_to_end = _arg;
    return *this;
  }
  Type & set__cross_track_error(
    const double & _arg)
  {
    this->cross_track_error = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    cola2_msgs::action::Pilot_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const cola2_msgs::action::Pilot_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cola2_msgs::action::Pilot_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cola2_msgs::action::Pilot_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::action::Pilot_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::action::Pilot_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::action::Pilot_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::action::Pilot_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cola2_msgs::action::Pilot_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cola2_msgs::action::Pilot_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cola2_msgs__action__Pilot_Feedback
    std::shared_ptr<cola2_msgs::action::Pilot_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cola2_msgs__action__Pilot_Feedback
    std::shared_ptr<cola2_msgs::action::Pilot_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Pilot_Feedback_ & other) const
  {
    if (this->distance_to_end != other.distance_to_end) {
      return false;
    }
    if (this->cross_track_error != other.cross_track_error) {
      return false;
    }
    return true;
  }
  bool operator!=(const Pilot_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Pilot_Feedback_

// alias to use template instance with default allocator
using Pilot_Feedback =
  cola2_msgs::action::Pilot_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace cola2_msgs


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "cola2_msgs/action/detail/pilot__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__cola2_msgs__action__Pilot_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__cola2_msgs__action__Pilot_SendGoal_Request __declspec(deprecated)
#endif

namespace cola2_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Pilot_SendGoal_Request_
{
  using Type = Pilot_SendGoal_Request_<ContainerAllocator>;

  explicit Pilot_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit Pilot_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    goal(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _goal_type =
    cola2_msgs::action::Pilot_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const cola2_msgs::action::Pilot_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    cola2_msgs::action::Pilot_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const cola2_msgs::action::Pilot_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cola2_msgs::action::Pilot_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cola2_msgs::action::Pilot_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::action::Pilot_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::action::Pilot_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::action::Pilot_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::action::Pilot_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cola2_msgs::action::Pilot_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cola2_msgs::action::Pilot_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cola2_msgs__action__Pilot_SendGoal_Request
    std::shared_ptr<cola2_msgs::action::Pilot_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cola2_msgs__action__Pilot_SendGoal_Request
    std::shared_ptr<cola2_msgs::action::Pilot_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Pilot_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const Pilot_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Pilot_SendGoal_Request_

// alias to use template instance with default allocator
using Pilot_SendGoal_Request =
  cola2_msgs::action::Pilot_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace cola2_msgs


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__cola2_msgs__action__Pilot_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__cola2_msgs__action__Pilot_SendGoal_Response __declspec(deprecated)
#endif

namespace cola2_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Pilot_SendGoal_Response_
{
  using Type = Pilot_SendGoal_Response_<ContainerAllocator>;

  explicit Pilot_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit Pilot_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    cola2_msgs::action::Pilot_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const cola2_msgs::action::Pilot_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cola2_msgs::action::Pilot_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cola2_msgs::action::Pilot_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::action::Pilot_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::action::Pilot_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::action::Pilot_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::action::Pilot_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cola2_msgs::action::Pilot_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cola2_msgs::action::Pilot_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cola2_msgs__action__Pilot_SendGoal_Response
    std::shared_ptr<cola2_msgs::action::Pilot_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cola2_msgs__action__Pilot_SendGoal_Response
    std::shared_ptr<cola2_msgs::action::Pilot_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Pilot_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const Pilot_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Pilot_SendGoal_Response_

// alias to use template instance with default allocator
using Pilot_SendGoal_Response =
  cola2_msgs::action::Pilot_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace cola2_msgs

namespace cola2_msgs
{

namespace action
{

struct Pilot_SendGoal
{
  using Request = cola2_msgs::action::Pilot_SendGoal_Request;
  using Response = cola2_msgs::action::Pilot_SendGoal_Response;
};

}  // namespace action

}  // namespace cola2_msgs


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__cola2_msgs__action__Pilot_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__cola2_msgs__action__Pilot_GetResult_Request __declspec(deprecated)
#endif

namespace cola2_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Pilot_GetResult_Request_
{
  using Type = Pilot_GetResult_Request_<ContainerAllocator>;

  explicit Pilot_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit Pilot_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    cola2_msgs::action::Pilot_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const cola2_msgs::action::Pilot_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cola2_msgs::action::Pilot_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cola2_msgs::action::Pilot_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::action::Pilot_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::action::Pilot_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::action::Pilot_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::action::Pilot_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cola2_msgs::action::Pilot_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cola2_msgs::action::Pilot_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cola2_msgs__action__Pilot_GetResult_Request
    std::shared_ptr<cola2_msgs::action::Pilot_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cola2_msgs__action__Pilot_GetResult_Request
    std::shared_ptr<cola2_msgs::action::Pilot_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Pilot_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const Pilot_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Pilot_GetResult_Request_

// alias to use template instance with default allocator
using Pilot_GetResult_Request =
  cola2_msgs::action::Pilot_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace cola2_msgs


// Include directives for member types
// Member 'result'
// already included above
// #include "cola2_msgs/action/detail/pilot__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__cola2_msgs__action__Pilot_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__cola2_msgs__action__Pilot_GetResult_Response __declspec(deprecated)
#endif

namespace cola2_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Pilot_GetResult_Response_
{
  using Type = Pilot_GetResult_Response_<ContainerAllocator>;

  explicit Pilot_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit Pilot_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    int8_t;
  _status_type status;
  using _result_type =
    cola2_msgs::action::Pilot_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const cola2_msgs::action::Pilot_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    cola2_msgs::action::Pilot_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const cola2_msgs::action::Pilot_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cola2_msgs::action::Pilot_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cola2_msgs::action::Pilot_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::action::Pilot_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::action::Pilot_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::action::Pilot_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::action::Pilot_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cola2_msgs::action::Pilot_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cola2_msgs::action::Pilot_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cola2_msgs__action__Pilot_GetResult_Response
    std::shared_ptr<cola2_msgs::action::Pilot_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cola2_msgs__action__Pilot_GetResult_Response
    std::shared_ptr<cola2_msgs::action::Pilot_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Pilot_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const Pilot_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Pilot_GetResult_Response_

// alias to use template instance with default allocator
using Pilot_GetResult_Response =
  cola2_msgs::action::Pilot_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace cola2_msgs

namespace cola2_msgs
{

namespace action
{

struct Pilot_GetResult
{
  using Request = cola2_msgs::action::Pilot_GetResult_Request;
  using Response = cola2_msgs::action::Pilot_GetResult_Response;
};

}  // namespace action

}  // namespace cola2_msgs


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "cola2_msgs/action/detail/pilot__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__cola2_msgs__action__Pilot_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__cola2_msgs__action__Pilot_FeedbackMessage __declspec(deprecated)
#endif

namespace cola2_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Pilot_FeedbackMessage_
{
  using Type = Pilot_FeedbackMessage_<ContainerAllocator>;

  explicit Pilot_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit Pilot_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    feedback(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _feedback_type =
    cola2_msgs::action::Pilot_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const cola2_msgs::action::Pilot_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    cola2_msgs::action::Pilot_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const cola2_msgs::action::Pilot_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cola2_msgs::action::Pilot_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cola2_msgs::action::Pilot_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::action::Pilot_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::action::Pilot_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cola2_msgs::action::Pilot_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cola2_msgs::action::Pilot_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cola2_msgs::action::Pilot_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cola2_msgs::action::Pilot_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cola2_msgs__action__Pilot_FeedbackMessage
    std::shared_ptr<cola2_msgs::action::Pilot_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cola2_msgs__action__Pilot_FeedbackMessage
    std::shared_ptr<cola2_msgs::action::Pilot_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Pilot_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const Pilot_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Pilot_FeedbackMessage_

// alias to use template instance with default allocator
using Pilot_FeedbackMessage =
  cola2_msgs::action::Pilot_FeedbackMessage_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace cola2_msgs

#include "action_msgs/srv/cancel_goal.hpp"
#include "action_msgs/msg/goal_info.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

namespace cola2_msgs
{

namespace action
{

struct Pilot
{
  /// The goal message defined in the action definition.
  using Goal = cola2_msgs::action::Pilot_Goal;
  /// The result message defined in the action definition.
  using Result = cola2_msgs::action::Pilot_Result;
  /// The feedback message defined in the action definition.
  using Feedback = cola2_msgs::action::Pilot_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = cola2_msgs::action::Pilot_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = cola2_msgs::action::Pilot_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = cola2_msgs::action::Pilot_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct Pilot Pilot;

}  // namespace action

}  // namespace cola2_msgs

#endif  // COLA2_MSGS__ACTION__DETAIL__PILOT__STRUCT_HPP_
