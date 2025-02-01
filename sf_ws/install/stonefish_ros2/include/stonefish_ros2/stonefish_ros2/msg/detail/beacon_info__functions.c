// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from stonefish_ros2:msg/BeaconInfo.idl
// generated code does not contain a copyright notice
#include "stonefish_ros2/msg/detail/beacon_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `relative_position`
#include "geometry_msgs/msg/detail/point__functions.h"
// Member `local_orientation`
#include "geometry_msgs/msg/detail/quaternion__functions.h"

bool
stonefish_ros2__msg__BeaconInfo__init(stonefish_ros2__msg__BeaconInfo * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    stonefish_ros2__msg__BeaconInfo__fini(msg);
    return false;
  }
  // beacon_id
  // range
  // azimuth
  // elevation
  // relative_position
  if (!geometry_msgs__msg__Point__init(&msg->relative_position)) {
    stonefish_ros2__msg__BeaconInfo__fini(msg);
    return false;
  }
  // local_orientation
  if (!geometry_msgs__msg__Quaternion__init(&msg->local_orientation)) {
    stonefish_ros2__msg__BeaconInfo__fini(msg);
    return false;
  }
  // local_depth
  return true;
}

void
stonefish_ros2__msg__BeaconInfo__fini(stonefish_ros2__msg__BeaconInfo * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // beacon_id
  // range
  // azimuth
  // elevation
  // relative_position
  geometry_msgs__msg__Point__fini(&msg->relative_position);
  // local_orientation
  geometry_msgs__msg__Quaternion__fini(&msg->local_orientation);
  // local_depth
}

bool
stonefish_ros2__msg__BeaconInfo__are_equal(const stonefish_ros2__msg__BeaconInfo * lhs, const stonefish_ros2__msg__BeaconInfo * rhs)
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
  // beacon_id
  if (lhs->beacon_id != rhs->beacon_id) {
    return false;
  }
  // range
  if (lhs->range != rhs->range) {
    return false;
  }
  // azimuth
  if (lhs->azimuth != rhs->azimuth) {
    return false;
  }
  // elevation
  if (lhs->elevation != rhs->elevation) {
    return false;
  }
  // relative_position
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->relative_position), &(rhs->relative_position)))
  {
    return false;
  }
  // local_orientation
  if (!geometry_msgs__msg__Quaternion__are_equal(
      &(lhs->local_orientation), &(rhs->local_orientation)))
  {
    return false;
  }
  // local_depth
  if (lhs->local_depth != rhs->local_depth) {
    return false;
  }
  return true;
}

bool
stonefish_ros2__msg__BeaconInfo__copy(
  const stonefish_ros2__msg__BeaconInfo * input,
  stonefish_ros2__msg__BeaconInfo * output)
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
  // beacon_id
  output->beacon_id = input->beacon_id;
  // range
  output->range = input->range;
  // azimuth
  output->azimuth = input->azimuth;
  // elevation
  output->elevation = input->elevation;
  // relative_position
  if (!geometry_msgs__msg__Point__copy(
      &(input->relative_position), &(output->relative_position)))
  {
    return false;
  }
  // local_orientation
  if (!geometry_msgs__msg__Quaternion__copy(
      &(input->local_orientation), &(output->local_orientation)))
  {
    return false;
  }
  // local_depth
  output->local_depth = input->local_depth;
  return true;
}

stonefish_ros2__msg__BeaconInfo *
stonefish_ros2__msg__BeaconInfo__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stonefish_ros2__msg__BeaconInfo * msg = (stonefish_ros2__msg__BeaconInfo *)allocator.allocate(sizeof(stonefish_ros2__msg__BeaconInfo), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(stonefish_ros2__msg__BeaconInfo));
  bool success = stonefish_ros2__msg__BeaconInfo__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
stonefish_ros2__msg__BeaconInfo__destroy(stonefish_ros2__msg__BeaconInfo * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    stonefish_ros2__msg__BeaconInfo__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
stonefish_ros2__msg__BeaconInfo__Sequence__init(stonefish_ros2__msg__BeaconInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stonefish_ros2__msg__BeaconInfo * data = NULL;

  if (size) {
    data = (stonefish_ros2__msg__BeaconInfo *)allocator.zero_allocate(size, sizeof(stonefish_ros2__msg__BeaconInfo), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = stonefish_ros2__msg__BeaconInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        stonefish_ros2__msg__BeaconInfo__fini(&data[i - 1]);
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
stonefish_ros2__msg__BeaconInfo__Sequence__fini(stonefish_ros2__msg__BeaconInfo__Sequence * array)
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
      stonefish_ros2__msg__BeaconInfo__fini(&array->data[i]);
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

stonefish_ros2__msg__BeaconInfo__Sequence *
stonefish_ros2__msg__BeaconInfo__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stonefish_ros2__msg__BeaconInfo__Sequence * array = (stonefish_ros2__msg__BeaconInfo__Sequence *)allocator.allocate(sizeof(stonefish_ros2__msg__BeaconInfo__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = stonefish_ros2__msg__BeaconInfo__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
stonefish_ros2__msg__BeaconInfo__Sequence__destroy(stonefish_ros2__msg__BeaconInfo__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    stonefish_ros2__msg__BeaconInfo__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
stonefish_ros2__msg__BeaconInfo__Sequence__are_equal(const stonefish_ros2__msg__BeaconInfo__Sequence * lhs, const stonefish_ros2__msg__BeaconInfo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!stonefish_ros2__msg__BeaconInfo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
stonefish_ros2__msg__BeaconInfo__Sequence__copy(
  const stonefish_ros2__msg__BeaconInfo__Sequence * input,
  stonefish_ros2__msg__BeaconInfo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(stonefish_ros2__msg__BeaconInfo);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    stonefish_ros2__msg__BeaconInfo * data =
      (stonefish_ros2__msg__BeaconInfo *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!stonefish_ros2__msg__BeaconInfo__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          stonefish_ros2__msg__BeaconInfo__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!stonefish_ros2__msg__BeaconInfo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
