// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from cola2_msgs:msg/Bool6Axis.idl
// generated code does not contain a copyright notice
#include "cola2_msgs/msg/detail/bool6_axis__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
cola2_msgs__msg__Bool6Axis__init(cola2_msgs__msg__Bool6Axis * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  // z
  // roll
  // pitch
  // yaw
  return true;
}

void
cola2_msgs__msg__Bool6Axis__fini(cola2_msgs__msg__Bool6Axis * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
  // z
  // roll
  // pitch
  // yaw
}

bool
cola2_msgs__msg__Bool6Axis__are_equal(const cola2_msgs__msg__Bool6Axis * lhs, const cola2_msgs__msg__Bool6Axis * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // z
  if (lhs->z != rhs->z) {
    return false;
  }
  // roll
  if (lhs->roll != rhs->roll) {
    return false;
  }
  // pitch
  if (lhs->pitch != rhs->pitch) {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  return true;
}

bool
cola2_msgs__msg__Bool6Axis__copy(
  const cola2_msgs__msg__Bool6Axis * input,
  cola2_msgs__msg__Bool6Axis * output)
{
  if (!input || !output) {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // z
  output->z = input->z;
  // roll
  output->roll = input->roll;
  // pitch
  output->pitch = input->pitch;
  // yaw
  output->yaw = input->yaw;
  return true;
}

cola2_msgs__msg__Bool6Axis *
cola2_msgs__msg__Bool6Axis__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__msg__Bool6Axis * msg = (cola2_msgs__msg__Bool6Axis *)allocator.allocate(sizeof(cola2_msgs__msg__Bool6Axis), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(cola2_msgs__msg__Bool6Axis));
  bool success = cola2_msgs__msg__Bool6Axis__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
cola2_msgs__msg__Bool6Axis__destroy(cola2_msgs__msg__Bool6Axis * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    cola2_msgs__msg__Bool6Axis__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
cola2_msgs__msg__Bool6Axis__Sequence__init(cola2_msgs__msg__Bool6Axis__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__msg__Bool6Axis * data = NULL;

  if (size) {
    data = (cola2_msgs__msg__Bool6Axis *)allocator.zero_allocate(size, sizeof(cola2_msgs__msg__Bool6Axis), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = cola2_msgs__msg__Bool6Axis__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        cola2_msgs__msg__Bool6Axis__fini(&data[i - 1]);
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
cola2_msgs__msg__Bool6Axis__Sequence__fini(cola2_msgs__msg__Bool6Axis__Sequence * array)
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
      cola2_msgs__msg__Bool6Axis__fini(&array->data[i]);
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

cola2_msgs__msg__Bool6Axis__Sequence *
cola2_msgs__msg__Bool6Axis__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__msg__Bool6Axis__Sequence * array = (cola2_msgs__msg__Bool6Axis__Sequence *)allocator.allocate(sizeof(cola2_msgs__msg__Bool6Axis__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = cola2_msgs__msg__Bool6Axis__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
cola2_msgs__msg__Bool6Axis__Sequence__destroy(cola2_msgs__msg__Bool6Axis__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    cola2_msgs__msg__Bool6Axis__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
cola2_msgs__msg__Bool6Axis__Sequence__are_equal(const cola2_msgs__msg__Bool6Axis__Sequence * lhs, const cola2_msgs__msg__Bool6Axis__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!cola2_msgs__msg__Bool6Axis__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
cola2_msgs__msg__Bool6Axis__Sequence__copy(
  const cola2_msgs__msg__Bool6Axis__Sequence * input,
  cola2_msgs__msg__Bool6Axis__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(cola2_msgs__msg__Bool6Axis);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    cola2_msgs__msg__Bool6Axis * data =
      (cola2_msgs__msg__Bool6Axis *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!cola2_msgs__msg__Bool6Axis__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          cola2_msgs__msg__Bool6Axis__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!cola2_msgs__msg__Bool6Axis__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
