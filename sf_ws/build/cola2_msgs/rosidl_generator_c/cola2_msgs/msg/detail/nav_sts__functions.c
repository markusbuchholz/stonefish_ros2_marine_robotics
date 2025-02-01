// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from cola2_msgs:msg/NavSts.idl
// generated code does not contain a copyright notice
#include "cola2_msgs/msg/detail/nav_sts__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `global_position`
// Member `origin`
#include "cola2_msgs/msg/detail/decimal_lat_lon__functions.h"
// Member `position`
// Member `position_variance`
#include "cola2_msgs/msg/detail/ned__functions.h"
// Member `body_velocity`
#include "geometry_msgs/msg/detail/point__functions.h"
// Member `orientation`
// Member `orientation_rate`
// Member `orientation_variance`
#include "cola2_msgs/msg/detail/rpy__functions.h"

bool
cola2_msgs__msg__NavSts__init(cola2_msgs__msg__NavSts * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    cola2_msgs__msg__NavSts__fini(msg);
    return false;
  }
  // global_position
  if (!cola2_msgs__msg__DecimalLatLon__init(&msg->global_position)) {
    cola2_msgs__msg__NavSts__fini(msg);
    return false;
  }
  // origin
  if (!cola2_msgs__msg__DecimalLatLon__init(&msg->origin)) {
    cola2_msgs__msg__NavSts__fini(msg);
    return false;
  }
  // position
  if (!cola2_msgs__msg__NED__init(&msg->position)) {
    cola2_msgs__msg__NavSts__fini(msg);
    return false;
  }
  // altitude
  // body_velocity
  if (!geometry_msgs__msg__Point__init(&msg->body_velocity)) {
    cola2_msgs__msg__NavSts__fini(msg);
    return false;
  }
  // orientation
  if (!cola2_msgs__msg__RPY__init(&msg->orientation)) {
    cola2_msgs__msg__NavSts__fini(msg);
    return false;
  }
  // orientation_rate
  if (!cola2_msgs__msg__RPY__init(&msg->orientation_rate)) {
    cola2_msgs__msg__NavSts__fini(msg);
    return false;
  }
  // position_variance
  if (!cola2_msgs__msg__NED__init(&msg->position_variance)) {
    cola2_msgs__msg__NavSts__fini(msg);
    return false;
  }
  // orientation_variance
  if (!cola2_msgs__msg__RPY__init(&msg->orientation_variance)) {
    cola2_msgs__msg__NavSts__fini(msg);
    return false;
  }
  return true;
}

void
cola2_msgs__msg__NavSts__fini(cola2_msgs__msg__NavSts * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // global_position
  cola2_msgs__msg__DecimalLatLon__fini(&msg->global_position);
  // origin
  cola2_msgs__msg__DecimalLatLon__fini(&msg->origin);
  // position
  cola2_msgs__msg__NED__fini(&msg->position);
  // altitude
  // body_velocity
  geometry_msgs__msg__Point__fini(&msg->body_velocity);
  // orientation
  cola2_msgs__msg__RPY__fini(&msg->orientation);
  // orientation_rate
  cola2_msgs__msg__RPY__fini(&msg->orientation_rate);
  // position_variance
  cola2_msgs__msg__NED__fini(&msg->position_variance);
  // orientation_variance
  cola2_msgs__msg__RPY__fini(&msg->orientation_variance);
}

bool
cola2_msgs__msg__NavSts__are_equal(const cola2_msgs__msg__NavSts * lhs, const cola2_msgs__msg__NavSts * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // global_position
  if (!cola2_msgs__msg__DecimalLatLon__are_equal(
      &(lhs->global_position), &(rhs->global_position)))
  {
    return false;
  }
  // origin
  if (!cola2_msgs__msg__DecimalLatLon__are_equal(
      &(lhs->origin), &(rhs->origin)))
  {
    return false;
  }
  // position
  if (!cola2_msgs__msg__NED__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  // altitude
  if (lhs->altitude != rhs->altitude) {
    return false;
  }
  // body_velocity
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->body_velocity), &(rhs->body_velocity)))
  {
    return false;
  }
  // orientation
  if (!cola2_msgs__msg__RPY__are_equal(
      &(lhs->orientation), &(rhs->orientation)))
  {
    return false;
  }
  // orientation_rate
  if (!cola2_msgs__msg__RPY__are_equal(
      &(lhs->orientation_rate), &(rhs->orientation_rate)))
  {
    return false;
  }
  // position_variance
  if (!cola2_msgs__msg__NED__are_equal(
      &(lhs->position_variance), &(rhs->position_variance)))
  {
    return false;
  }
  // orientation_variance
  if (!cola2_msgs__msg__RPY__are_equal(
      &(lhs->orientation_variance), &(rhs->orientation_variance)))
  {
    return false;
  }
  return true;
}

bool
cola2_msgs__msg__NavSts__copy(
  const cola2_msgs__msg__NavSts * input,
  cola2_msgs__msg__NavSts * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // global_position
  if (!cola2_msgs__msg__DecimalLatLon__copy(
      &(input->global_position), &(output->global_position)))
  {
    return false;
  }
  // origin
  if (!cola2_msgs__msg__DecimalLatLon__copy(
      &(input->origin), &(output->origin)))
  {
    return false;
  }
  // position
  if (!cola2_msgs__msg__NED__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  // altitude
  output->altitude = input->altitude;
  // body_velocity
  if (!geometry_msgs__msg__Point__copy(
      &(input->body_velocity), &(output->body_velocity)))
  {
    return false;
  }
  // orientation
  if (!cola2_msgs__msg__RPY__copy(
      &(input->orientation), &(output->orientation)))
  {
    return false;
  }
  // orientation_rate
  if (!cola2_msgs__msg__RPY__copy(
      &(input->orientation_rate), &(output->orientation_rate)))
  {
    return false;
  }
  // position_variance
  if (!cola2_msgs__msg__NED__copy(
      &(input->position_variance), &(output->position_variance)))
  {
    return false;
  }
  // orientation_variance
  if (!cola2_msgs__msg__RPY__copy(
      &(input->orientation_variance), &(output->orientation_variance)))
  {
    return false;
  }
  return true;
}

cola2_msgs__msg__NavSts *
cola2_msgs__msg__NavSts__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__msg__NavSts * msg = (cola2_msgs__msg__NavSts *)allocator.allocate(sizeof(cola2_msgs__msg__NavSts), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(cola2_msgs__msg__NavSts));
  bool success = cola2_msgs__msg__NavSts__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
cola2_msgs__msg__NavSts__destroy(cola2_msgs__msg__NavSts * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    cola2_msgs__msg__NavSts__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
cola2_msgs__msg__NavSts__Sequence__init(cola2_msgs__msg__NavSts__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__msg__NavSts * data = NULL;

  if (size) {
    data = (cola2_msgs__msg__NavSts *)allocator.zero_allocate(size, sizeof(cola2_msgs__msg__NavSts), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = cola2_msgs__msg__NavSts__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        cola2_msgs__msg__NavSts__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
cola2_msgs__msg__NavSts__Sequence__fini(cola2_msgs__msg__NavSts__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      cola2_msgs__msg__NavSts__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

cola2_msgs__msg__NavSts__Sequence *
cola2_msgs__msg__NavSts__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__msg__NavSts__Sequence * array = (cola2_msgs__msg__NavSts__Sequence *)allocator.allocate(sizeof(cola2_msgs__msg__NavSts__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = cola2_msgs__msg__NavSts__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
cola2_msgs__msg__NavSts__Sequence__destroy(cola2_msgs__msg__NavSts__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    cola2_msgs__msg__NavSts__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
cola2_msgs__msg__NavSts__Sequence__are_equal(const cola2_msgs__msg__NavSts__Sequence * lhs, const cola2_msgs__msg__NavSts__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!cola2_msgs__msg__NavSts__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
cola2_msgs__msg__NavSts__Sequence__copy(
  const cola2_msgs__msg__NavSts__Sequence * input,
  cola2_msgs__msg__NavSts__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(cola2_msgs__msg__NavSts);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    cola2_msgs__msg__NavSts * data =
      (cola2_msgs__msg__NavSts *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!cola2_msgs__msg__NavSts__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          cola2_msgs__msg__NavSts__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!cola2_msgs__msg__NavSts__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
