// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from cola2_msgs:msg/MissionState.idl
// generated code does not contain a copyright notice
#include "cola2_msgs/msg/detail/mission_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `name`
#include "rosidl_runtime_c/string_functions.h"
// Member `last_active`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
cola2_msgs__msg__MissionState__init(cola2_msgs__msg__MissionState * msg)
{
  if (!msg) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__init(&msg->name)) {
    cola2_msgs__msg__MissionState__fini(msg);
    return false;
  }
  // current_step
  // last_active
  if (!builtin_interfaces__msg__Time__init(&msg->last_active)) {
    cola2_msgs__msg__MissionState__fini(msg);
    return false;
  }
  return true;
}

void
cola2_msgs__msg__MissionState__fini(cola2_msgs__msg__MissionState * msg)
{
  if (!msg) {
    return;
  }
  // name
  rosidl_runtime_c__String__fini(&msg->name);
  // current_step
  // last_active
  builtin_interfaces__msg__Time__fini(&msg->last_active);
}

bool
cola2_msgs__msg__MissionState__are_equal(const cola2_msgs__msg__MissionState * lhs, const cola2_msgs__msg__MissionState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->name), &(rhs->name)))
  {
    return false;
  }
  // current_step
  if (lhs->current_step != rhs->current_step) {
    return false;
  }
  // last_active
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->last_active), &(rhs->last_active)))
  {
    return false;
  }
  return true;
}

bool
cola2_msgs__msg__MissionState__copy(
  const cola2_msgs__msg__MissionState * input,
  cola2_msgs__msg__MissionState * output)
{
  if (!input || !output) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__copy(
      &(input->name), &(output->name)))
  {
    return false;
  }
  // current_step
  output->current_step = input->current_step;
  // last_active
  if (!builtin_interfaces__msg__Time__copy(
      &(input->last_active), &(output->last_active)))
  {
    return false;
  }
  return true;
}

cola2_msgs__msg__MissionState *
cola2_msgs__msg__MissionState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__msg__MissionState * msg = (cola2_msgs__msg__MissionState *)allocator.allocate(sizeof(cola2_msgs__msg__MissionState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(cola2_msgs__msg__MissionState));
  bool success = cola2_msgs__msg__MissionState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
cola2_msgs__msg__MissionState__destroy(cola2_msgs__msg__MissionState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    cola2_msgs__msg__MissionState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
cola2_msgs__msg__MissionState__Sequence__init(cola2_msgs__msg__MissionState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__msg__MissionState * data = NULL;

  if (size) {
    data = (cola2_msgs__msg__MissionState *)allocator.zero_allocate(size, sizeof(cola2_msgs__msg__MissionState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = cola2_msgs__msg__MissionState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        cola2_msgs__msg__MissionState__fini(&data[i - 1]);
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
cola2_msgs__msg__MissionState__Sequence__fini(cola2_msgs__msg__MissionState__Sequence * array)
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
      cola2_msgs__msg__MissionState__fini(&array->data[i]);
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

cola2_msgs__msg__MissionState__Sequence *
cola2_msgs__msg__MissionState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__msg__MissionState__Sequence * array = (cola2_msgs__msg__MissionState__Sequence *)allocator.allocate(sizeof(cola2_msgs__msg__MissionState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = cola2_msgs__msg__MissionState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
cola2_msgs__msg__MissionState__Sequence__destroy(cola2_msgs__msg__MissionState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    cola2_msgs__msg__MissionState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
cola2_msgs__msg__MissionState__Sequence__are_equal(const cola2_msgs__msg__MissionState__Sequence * lhs, const cola2_msgs__msg__MissionState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!cola2_msgs__msg__MissionState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
cola2_msgs__msg__MissionState__Sequence__copy(
  const cola2_msgs__msg__MissionState__Sequence * input,
  cola2_msgs__msg__MissionState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(cola2_msgs__msg__MissionState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    cola2_msgs__msg__MissionState * data =
      (cola2_msgs__msg__MissionState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!cola2_msgs__msg__MissionState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          cola2_msgs__msg__MissionState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!cola2_msgs__msg__MissionState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
