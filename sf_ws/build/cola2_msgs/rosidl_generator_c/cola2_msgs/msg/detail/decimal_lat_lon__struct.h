// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from cola2_msgs:msg/DecimalLatLon.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__DECIMAL_LAT_LON__STRUCT_H_
#define COLA2_MSGS__MSG__DETAIL__DECIMAL_LAT_LON__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/DecimalLatLon in the package cola2_msgs.
/**
  * Latitude and longitude, in decimal form.
 */
typedef struct cola2_msgs__msg__DecimalLatLon
{
  double latitude;
  double longitude;
} cola2_msgs__msg__DecimalLatLon;

// Struct for a sequence of cola2_msgs__msg__DecimalLatLon.
typedef struct cola2_msgs__msg__DecimalLatLon__Sequence
{
  cola2_msgs__msg__DecimalLatLon * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cola2_msgs__msg__DecimalLatLon__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // COLA2_MSGS__MSG__DETAIL__DECIMAL_LAT_LON__STRUCT_H_
