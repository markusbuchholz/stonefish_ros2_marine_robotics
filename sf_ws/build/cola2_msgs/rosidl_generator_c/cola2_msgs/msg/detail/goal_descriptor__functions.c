// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from cola2_msgs:msg/GoalDescriptor.idl
// generated code does not contain a copyright notice
#include "cola2_msgs/msg/detail/goal_descriptor__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `requester`
#include "rosidl_runtime_c/string_functions.h"

bool
cola2_msgs__msg__GoalDescriptor__init(cola2_msgs__msg__GoalDescriptor * msg)
{
  if (!msg) {
    return false;
  }
  // requester
  if (!rosidl_runtime_c__String__init(&msg->requester)) {
    cola2_msgs__msg__GoalDescriptor__fini(msg);
    return false;
  }
  // priority
  return true;
}

void
cola2_msgs__msg__GoalDescriptor__fini(cola2_msgs__msg__GoalDescriptor * msg)
{
  if (!msg) {
    return;
  }
  // requester
  rosidl_runtime_c__String__fini(&msg->requester);
  // priority
}

bool
cola2_msgs__msg__GoalDescriptor__are_equal(const cola2_msgs__msg__GoalDescriptor * lhs, const cola2_msgs__msg__GoalDescriptor * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // requester
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->requester), &(rhs->requester)))
  {
    return false;
  }
  // priority
  if (lhs->priority != rhs->priority) {
    return false;
  }
  return true;
}

bool
cola2_msgs__msg__GoalDescriptor__copy(
  const cola2_msgs__msg__GoalDescriptor * input,
  cola2_msgs__msg__GoalDescriptor * output)
{
  if (!input || !output) {
    return false;
  }
  // requester
  if (!rosidl_runtime_c__String__copy(
      &(input->requester), &(output->requester)))
  {
    return false;
  }
  // priority
  output->priority = input->priority;
  return true;
}

cola2_msgs__msg__GoalDescriptor *
cola2_msgs__msg__GoalDescriptor__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__msg__GoalDescriptor * msg = (cola2_msgs__msg__GoalDescriptor *)allocator.allocate(sizeof(cola2_msgs__msg__GoalDescriptor), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(cola2_msgs__msg__GoalDescriptor));
  bool success = cola2_msgs__msg__GoalDescriptor__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
cola2_msgs__msg__GoalDescriptor__destroy(cola2_msgs__msg__GoalDescriptor * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    cola2_msgs__msg__GoalDescriptor__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
cola2_msgs__msg__GoalDescriptor__Sequence__init(cola2_msgs__msg__GoalDescriptor__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__msg__GoalDescriptor * data = NULL;

  if (size) {
    data = (cola2_msgs__msg__GoalDescriptor *)allocator.zero_allocate(size, sizeof(cola2_msgs__msg__GoalDescriptor), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = cola2_msgs__msg__GoalDescriptor__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        cola2_msgs__msg__GoalDescriptor__fini(&data[i - 1]);
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
cola2_msgs__msg__GoalDescriptor__Sequence__fini(cola2_msgs__msg__GoalDescriptor__Sequence * array)
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
      cola2_msgs__msg__GoalDescriptor__fini(&array->data[i]);
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

cola2_msgs__msg__GoalDescriptor__Sequence *
cola2_msgs__msg__GoalDescriptor__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__msg__GoalDescriptor__Sequence * array = (cola2_msgs__msg__GoalDescriptor__Sequence *)allocator.allocate(sizeof(cola2_msgs__msg__GoalDescriptor__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = cola2_msgs__msg__GoalDescriptor__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
cola2_msgs__msg__GoalDescriptor__Sequence__destroy(cola2_msgs__msg__GoalDescriptor__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    cola2_msgs__msg__GoalDescriptor__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
cola2_msgs__msg__GoalDescriptor__Sequence__are_equal(const cola2_msgs__msg__GoalDescriptor__Sequence * lhs, const cola2_msgs__msg__GoalDescriptor__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!cola2_msgs__msg__GoalDescriptor__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
cola2_msgs__msg__GoalDescriptor__Sequence__copy(
  const cola2_msgs__msg__GoalDescriptor__Sequence * input,
  cola2_msgs__msg__GoalDescriptor__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(cola2_msgs__msg__GoalDescriptor);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    cola2_msgs__msg__GoalDescriptor * data =
      (cola2_msgs__msg__GoalDescriptor *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!cola2_msgs__msg__GoalDescriptor__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          cola2_msgs__msg__GoalDescriptor__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!cola2_msgs__msg__GoalDescriptor__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
