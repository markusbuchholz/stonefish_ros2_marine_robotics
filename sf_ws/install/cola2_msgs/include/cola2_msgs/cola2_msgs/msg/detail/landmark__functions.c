// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from cola2_msgs:msg/Landmark.idl
// generated code does not contain a copyright notice
#include "cola2_msgs/msg/detail/landmark__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `last_update`
#include "builtin_interfaces/msg/detail/time__functions.h"
// Member `landmark_id`
#include "rosidl_runtime_c/string_functions.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose_with_covariance__functions.h"

bool
cola2_msgs__msg__Landmark__init(cola2_msgs__msg__Landmark * msg)
{
  if (!msg) {
    return false;
  }
  // last_update
  if (!builtin_interfaces__msg__Time__init(&msg->last_update)) {
    cola2_msgs__msg__Landmark__fini(msg);
    return false;
  }
  // landmark_id
  if (!rosidl_runtime_c__String__init(&msg->landmark_id)) {
    cola2_msgs__msg__Landmark__fini(msg);
    return false;
  }
  // pose
  if (!geometry_msgs__msg__PoseWithCovariance__init(&msg->pose)) {
    cola2_msgs__msg__Landmark__fini(msg);
    return false;
  }
  return true;
}

void
cola2_msgs__msg__Landmark__fini(cola2_msgs__msg__Landmark * msg)
{
  if (!msg) {
    return;
  }
  // last_update
  builtin_interfaces__msg__Time__fini(&msg->last_update);
  // landmark_id
  rosidl_runtime_c__String__fini(&msg->landmark_id);
  // pose
  geometry_msgs__msg__PoseWithCovariance__fini(&msg->pose);
}

bool
cola2_msgs__msg__Landmark__are_equal(const cola2_msgs__msg__Landmark * lhs, const cola2_msgs__msg__Landmark * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // last_update
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->last_update), &(rhs->last_update)))
  {
    return false;
  }
  // landmark_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->landmark_id), &(rhs->landmark_id)))
  {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__PoseWithCovariance__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  return true;
}

bool
cola2_msgs__msg__Landmark__copy(
  const cola2_msgs__msg__Landmark * input,
  cola2_msgs__msg__Landmark * output)
{
  if (!input || !output) {
    return false;
  }
  // last_update
  if (!builtin_interfaces__msg__Time__copy(
      &(input->last_update), &(output->last_update)))
  {
    return false;
  }
  // landmark_id
  if (!rosidl_runtime_c__String__copy(
      &(input->landmark_id), &(output->landmark_id)))
  {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__PoseWithCovariance__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  return true;
}

cola2_msgs__msg__Landmark *
cola2_msgs__msg__Landmark__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__msg__Landmark * msg = (cola2_msgs__msg__Landmark *)allocator.allocate(sizeof(cola2_msgs__msg__Landmark), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(cola2_msgs__msg__Landmark));
  bool success = cola2_msgs__msg__Landmark__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
cola2_msgs__msg__Landmark__destroy(cola2_msgs__msg__Landmark * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    cola2_msgs__msg__Landmark__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
cola2_msgs__msg__Landmark__Sequence__init(cola2_msgs__msg__Landmark__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__msg__Landmark * data = NULL;

  if (size) {
    data = (cola2_msgs__msg__Landmark *)allocator.zero_allocate(size, sizeof(cola2_msgs__msg__Landmark), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = cola2_msgs__msg__Landmark__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        cola2_msgs__msg__Landmark__fini(&data[i - 1]);
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
cola2_msgs__msg__Landmark__Sequence__fini(cola2_msgs__msg__Landmark__Sequence * array)
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
      cola2_msgs__msg__Landmark__fini(&array->data[i]);
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

cola2_msgs__msg__Landmark__Sequence *
cola2_msgs__msg__Landmark__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__msg__Landmark__Sequence * array = (cola2_msgs__msg__Landmark__Sequence *)allocator.allocate(sizeof(cola2_msgs__msg__Landmark__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = cola2_msgs__msg__Landmark__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
cola2_msgs__msg__Landmark__Sequence__destroy(cola2_msgs__msg__Landmark__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    cola2_msgs__msg__Landmark__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
cola2_msgs__msg__Landmark__Sequence__are_equal(const cola2_msgs__msg__Landmark__Sequence * lhs, const cola2_msgs__msg__Landmark__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!cola2_msgs__msg__Landmark__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
cola2_msgs__msg__Landmark__Sequence__copy(
  const cola2_msgs__msg__Landmark__Sequence * input,
  cola2_msgs__msg__Landmark__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(cola2_msgs__msg__Landmark);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    cola2_msgs__msg__Landmark * data =
      (cola2_msgs__msg__Landmark *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!cola2_msgs__msg__Landmark__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          cola2_msgs__msg__Landmark__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!cola2_msgs__msg__Landmark__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
