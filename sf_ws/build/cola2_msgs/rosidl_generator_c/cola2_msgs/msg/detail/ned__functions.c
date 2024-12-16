// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from cola2_msgs:msg/NED.idl
// generated code does not contain a copyright notice
#include "cola2_msgs/msg/detail/ned__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
cola2_msgs__msg__NED__init(cola2_msgs__msg__NED * msg)
{
  if (!msg) {
    return false;
  }
  // north
  // east
  // depth
  return true;
}

void
cola2_msgs__msg__NED__fini(cola2_msgs__msg__NED * msg)
{
  if (!msg) {
    return;
  }
  // north
  // east
  // depth
}

bool
cola2_msgs__msg__NED__are_equal(const cola2_msgs__msg__NED * lhs, const cola2_msgs__msg__NED * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // north
  if (lhs->north != rhs->north) {
    return false;
  }
  // east
  if (lhs->east != rhs->east) {
    return false;
  }
  // depth
  if (lhs->depth != rhs->depth) {
    return false;
  }
  return true;
}

bool
cola2_msgs__msg__NED__copy(
  const cola2_msgs__msg__NED * input,
  cola2_msgs__msg__NED * output)
{
  if (!input || !output) {
    return false;
  }
  // north
  output->north = input->north;
  // east
  output->east = input->east;
  // depth
  output->depth = input->depth;
  return true;
}

cola2_msgs__msg__NED *
cola2_msgs__msg__NED__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__msg__NED * msg = (cola2_msgs__msg__NED *)allocator.allocate(sizeof(cola2_msgs__msg__NED), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(cola2_msgs__msg__NED));
  bool success = cola2_msgs__msg__NED__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
cola2_msgs__msg__NED__destroy(cola2_msgs__msg__NED * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    cola2_msgs__msg__NED__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
cola2_msgs__msg__NED__Sequence__init(cola2_msgs__msg__NED__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__msg__NED * data = NULL;

  if (size) {
    data = (cola2_msgs__msg__NED *)allocator.zero_allocate(size, sizeof(cola2_msgs__msg__NED), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = cola2_msgs__msg__NED__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        cola2_msgs__msg__NED__fini(&data[i - 1]);
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
cola2_msgs__msg__NED__Sequence__fini(cola2_msgs__msg__NED__Sequence * array)
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
      cola2_msgs__msg__NED__fini(&array->data[i]);
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

cola2_msgs__msg__NED__Sequence *
cola2_msgs__msg__NED__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__msg__NED__Sequence * array = (cola2_msgs__msg__NED__Sequence *)allocator.allocate(sizeof(cola2_msgs__msg__NED__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = cola2_msgs__msg__NED__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
cola2_msgs__msg__NED__Sequence__destroy(cola2_msgs__msg__NED__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    cola2_msgs__msg__NED__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
cola2_msgs__msg__NED__Sequence__are_equal(const cola2_msgs__msg__NED__Sequence * lhs, const cola2_msgs__msg__NED__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!cola2_msgs__msg__NED__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
cola2_msgs__msg__NED__Sequence__copy(
  const cola2_msgs__msg__NED__Sequence * input,
  cola2_msgs__msg__NED__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(cola2_msgs__msg__NED);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    cola2_msgs__msg__NED * data =
      (cola2_msgs__msg__NED *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!cola2_msgs__msg__NED__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          cola2_msgs__msg__NED__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!cola2_msgs__msg__NED__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
