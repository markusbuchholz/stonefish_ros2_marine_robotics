// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from cola2_msgs:msg/BodyForceReq.idl
// generated code does not contain a copyright notice
#include "cola2_msgs/msg/detail/body_force_req__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `goal`
#include "cola2_msgs/msg/detail/goal_descriptor__functions.h"
// Member `wrench`
#include "geometry_msgs/msg/detail/wrench__functions.h"
// Member `disable_axis`
#include "cola2_msgs/msg/detail/bool6_axis__functions.h"

bool
cola2_msgs__msg__BodyForceReq__init(cola2_msgs__msg__BodyForceReq * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    cola2_msgs__msg__BodyForceReq__fini(msg);
    return false;
  }
  // goal
  if (!cola2_msgs__msg__GoalDescriptor__init(&msg->goal)) {
    cola2_msgs__msg__BodyForceReq__fini(msg);
    return false;
  }
  // wrench
  if (!geometry_msgs__msg__Wrench__init(&msg->wrench)) {
    cola2_msgs__msg__BodyForceReq__fini(msg);
    return false;
  }
  // disable_axis
  if (!cola2_msgs__msg__Bool6Axis__init(&msg->disable_axis)) {
    cola2_msgs__msg__BodyForceReq__fini(msg);
    return false;
  }
  return true;
}

void
cola2_msgs__msg__BodyForceReq__fini(cola2_msgs__msg__BodyForceReq * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // goal
  cola2_msgs__msg__GoalDescriptor__fini(&msg->goal);
  // wrench
  geometry_msgs__msg__Wrench__fini(&msg->wrench);
  // disable_axis
  cola2_msgs__msg__Bool6Axis__fini(&msg->disable_axis);
}

bool
cola2_msgs__msg__BodyForceReq__are_equal(const cola2_msgs__msg__BodyForceReq * lhs, const cola2_msgs__msg__BodyForceReq * rhs)
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
  // goal
  if (!cola2_msgs__msg__GoalDescriptor__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  // wrench
  if (!geometry_msgs__msg__Wrench__are_equal(
      &(lhs->wrench), &(rhs->wrench)))
  {
    return false;
  }
  // disable_axis
  if (!cola2_msgs__msg__Bool6Axis__are_equal(
      &(lhs->disable_axis), &(rhs->disable_axis)))
  {
    return false;
  }
  return true;
}

bool
cola2_msgs__msg__BodyForceReq__copy(
  const cola2_msgs__msg__BodyForceReq * input,
  cola2_msgs__msg__BodyForceReq * output)
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
  // goal
  if (!cola2_msgs__msg__GoalDescriptor__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  // wrench
  if (!geometry_msgs__msg__Wrench__copy(
      &(input->wrench), &(output->wrench)))
  {
    return false;
  }
  // disable_axis
  if (!cola2_msgs__msg__Bool6Axis__copy(
      &(input->disable_axis), &(output->disable_axis)))
  {
    return false;
  }
  return true;
}

cola2_msgs__msg__BodyForceReq *
cola2_msgs__msg__BodyForceReq__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__msg__BodyForceReq * msg = (cola2_msgs__msg__BodyForceReq *)allocator.allocate(sizeof(cola2_msgs__msg__BodyForceReq), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(cola2_msgs__msg__BodyForceReq));
  bool success = cola2_msgs__msg__BodyForceReq__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
cola2_msgs__msg__BodyForceReq__destroy(cola2_msgs__msg__BodyForceReq * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    cola2_msgs__msg__BodyForceReq__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
cola2_msgs__msg__BodyForceReq__Sequence__init(cola2_msgs__msg__BodyForceReq__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__msg__BodyForceReq * data = NULL;

  if (size) {
    data = (cola2_msgs__msg__BodyForceReq *)allocator.zero_allocate(size, sizeof(cola2_msgs__msg__BodyForceReq), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = cola2_msgs__msg__BodyForceReq__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        cola2_msgs__msg__BodyForceReq__fini(&data[i - 1]);
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
cola2_msgs__msg__BodyForceReq__Sequence__fini(cola2_msgs__msg__BodyForceReq__Sequence * array)
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
      cola2_msgs__msg__BodyForceReq__fini(&array->data[i]);
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

cola2_msgs__msg__BodyForceReq__Sequence *
cola2_msgs__msg__BodyForceReq__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__msg__BodyForceReq__Sequence * array = (cola2_msgs__msg__BodyForceReq__Sequence *)allocator.allocate(sizeof(cola2_msgs__msg__BodyForceReq__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = cola2_msgs__msg__BodyForceReq__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
cola2_msgs__msg__BodyForceReq__Sequence__destroy(cola2_msgs__msg__BodyForceReq__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    cola2_msgs__msg__BodyForceReq__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
cola2_msgs__msg__BodyForceReq__Sequence__are_equal(const cola2_msgs__msg__BodyForceReq__Sequence * lhs, const cola2_msgs__msg__BodyForceReq__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!cola2_msgs__msg__BodyForceReq__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
cola2_msgs__msg__BodyForceReq__Sequence__copy(
  const cola2_msgs__msg__BodyForceReq__Sequence * input,
  cola2_msgs__msg__BodyForceReq__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(cola2_msgs__msg__BodyForceReq);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    cola2_msgs__msg__BodyForceReq * data =
      (cola2_msgs__msg__BodyForceReq *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!cola2_msgs__msg__BodyForceReq__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          cola2_msgs__msg__BodyForceReq__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!cola2_msgs__msg__BodyForceReq__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
