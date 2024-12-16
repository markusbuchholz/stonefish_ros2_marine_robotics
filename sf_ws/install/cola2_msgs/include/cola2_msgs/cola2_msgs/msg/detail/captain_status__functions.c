// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from cola2_msgs:msg/CaptainStatus.idl
// generated code does not contain a copyright notice
#include "cola2_msgs/msg/detail/captain_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `message`
#include "rosidl_runtime_c/string_functions.h"
// Member `loaded_missions`
#include "cola2_msgs/msg/detail/mission_state__functions.h"

bool
cola2_msgs__msg__CaptainStatus__init(cola2_msgs__msg__CaptainStatus * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    cola2_msgs__msg__CaptainStatus__fini(msg);
    return false;
  }
  // state
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    cola2_msgs__msg__CaptainStatus__fini(msg);
    return false;
  }
  // loaded_missions
  if (!cola2_msgs__msg__MissionState__Sequence__init(&msg->loaded_missions, 0)) {
    cola2_msgs__msg__CaptainStatus__fini(msg);
    return false;
  }
  return true;
}

void
cola2_msgs__msg__CaptainStatus__fini(cola2_msgs__msg__CaptainStatus * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // state
  // message
  rosidl_runtime_c__String__fini(&msg->message);
  // loaded_missions
  cola2_msgs__msg__MissionState__Sequence__fini(&msg->loaded_missions);
}

bool
cola2_msgs__msg__CaptainStatus__are_equal(const cola2_msgs__msg__CaptainStatus * lhs, const cola2_msgs__msg__CaptainStatus * rhs)
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
  // state
  if (lhs->state != rhs->state) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  // loaded_missions
  if (!cola2_msgs__msg__MissionState__Sequence__are_equal(
      &(lhs->loaded_missions), &(rhs->loaded_missions)))
  {
    return false;
  }
  return true;
}

bool
cola2_msgs__msg__CaptainStatus__copy(
  const cola2_msgs__msg__CaptainStatus * input,
  cola2_msgs__msg__CaptainStatus * output)
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
  // state
  output->state = input->state;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  // loaded_missions
  if (!cola2_msgs__msg__MissionState__Sequence__copy(
      &(input->loaded_missions), &(output->loaded_missions)))
  {
    return false;
  }
  return true;
}

cola2_msgs__msg__CaptainStatus *
cola2_msgs__msg__CaptainStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__msg__CaptainStatus * msg = (cola2_msgs__msg__CaptainStatus *)allocator.allocate(sizeof(cola2_msgs__msg__CaptainStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(cola2_msgs__msg__CaptainStatus));
  bool success = cola2_msgs__msg__CaptainStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
cola2_msgs__msg__CaptainStatus__destroy(cola2_msgs__msg__CaptainStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    cola2_msgs__msg__CaptainStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
cola2_msgs__msg__CaptainStatus__Sequence__init(cola2_msgs__msg__CaptainStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__msg__CaptainStatus * data = NULL;

  if (size) {
    data = (cola2_msgs__msg__CaptainStatus *)allocator.zero_allocate(size, sizeof(cola2_msgs__msg__CaptainStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = cola2_msgs__msg__CaptainStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        cola2_msgs__msg__CaptainStatus__fini(&data[i - 1]);
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
cola2_msgs__msg__CaptainStatus__Sequence__fini(cola2_msgs__msg__CaptainStatus__Sequence * array)
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
      cola2_msgs__msg__CaptainStatus__fini(&array->data[i]);
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

cola2_msgs__msg__CaptainStatus__Sequence *
cola2_msgs__msg__CaptainStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__msg__CaptainStatus__Sequence * array = (cola2_msgs__msg__CaptainStatus__Sequence *)allocator.allocate(sizeof(cola2_msgs__msg__CaptainStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = cola2_msgs__msg__CaptainStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
cola2_msgs__msg__CaptainStatus__Sequence__destroy(cola2_msgs__msg__CaptainStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    cola2_msgs__msg__CaptainStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
cola2_msgs__msg__CaptainStatus__Sequence__are_equal(const cola2_msgs__msg__CaptainStatus__Sequence * lhs, const cola2_msgs__msg__CaptainStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!cola2_msgs__msg__CaptainStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
cola2_msgs__msg__CaptainStatus__Sequence__copy(
  const cola2_msgs__msg__CaptainStatus__Sequence * input,
  cola2_msgs__msg__CaptainStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(cola2_msgs__msg__CaptainStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    cola2_msgs__msg__CaptainStatus * data =
      (cola2_msgs__msg__CaptainStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!cola2_msgs__msg__CaptainStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          cola2_msgs__msg__CaptainStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!cola2_msgs__msg__CaptainStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
