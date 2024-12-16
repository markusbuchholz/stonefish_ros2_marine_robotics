// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from cola2_msgs:msg/DecimalLatLon.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__DECIMAL_LAT_LON__BUILDER_HPP_
#define COLA2_MSGS__MSG__DETAIL__DECIMAL_LAT_LON__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "cola2_msgs/msg/detail/decimal_lat_lon__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace cola2_msgs
{

namespace msg
{

namespace builder
{

class Init_DecimalLatLon_longitude
{
public:
  explicit Init_DecimalLatLon_longitude(::cola2_msgs::msg::DecimalLatLon & msg)
  : msg_(msg)
  {}
  ::cola2_msgs::msg::DecimalLatLon longitude(::cola2_msgs::msg::DecimalLatLon::_longitude_type arg)
  {
    msg_.longitude = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cola2_msgs::msg::DecimalLatLon msg_;
};

class Init_DecimalLatLon_latitude
{
public:
  Init_DecimalLatLon_latitude()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DecimalLatLon_longitude latitude(::cola2_msgs::msg::DecimalLatLon::_latitude_type arg)
  {
    msg_.latitude = std::move(arg);
    return Init_DecimalLatLon_longitude(msg_);
  }

private:
  ::cola2_msgs::msg::DecimalLatLon msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::cola2_msgs::msg::DecimalLatLon>()
{
  return cola2_msgs::msg::builder::Init_DecimalLatLon_latitude();
}

}  // namespace cola2_msgs

#endif  // COLA2_MSGS__MSG__DETAIL__DECIMAL_LAT_LON__BUILDER_HPP_
