// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from stonefish_ros2:msg/NEDPose.idl
// generated code does not contain a copyright notice
#include "stonefish_ros2/msg/detail/ned_pose__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
stonefish_ros2__msg__NEDPose__init(stonefish_ros2__msg__NEDPose * msg)
{
  if (!msg) {
    return false;
  }
  // north
  // east
  // down
  // roll
  // pitch
  // yaw
  return true;
}

void
stonefish_ros2__msg__NEDPose__fini(stonefish_ros2__msg__NEDPose * msg)
{
  if (!msg) {
    return;
  }
  // north
  // east
  // down
  // roll
  // pitch
  // yaw
}

bool
stonefish_ros2__msg__NEDPose__are_equal(const stonefish_ros2__msg__NEDPose * lhs, const stonefish_ros2__msg__NEDPose * rhs)
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
  // down
  if (lhs->down != rhs->down) {
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
stonefish_ros2__msg__NEDPose__copy(
  const stonefish_ros2__msg__NEDPose * input,
  stonefish_ros2__msg__NEDPose * output)
{
  if (!input || !output) {
    return false;
  }
  // north
  output->north = input->north;
  // east
  output->east = input->east;
  // down
  output->down = input->down;
  // roll
  output->roll = input->roll;
  // pitch
  output->pitch = input->pitch;
  // yaw
  output->yaw = input->yaw;
  return true;
}

stonefish_ros2__msg__NEDPose *
stonefish_ros2__msg__NEDPose__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stonefish_ros2__msg__NEDPose * msg = (stonefish_ros2__msg__NEDPose *)allocator.allocate(sizeof(stonefish_ros2__msg__NEDPose), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(stonefish_ros2__msg__NEDPose));
  bool success = stonefish_ros2__msg__NEDPose__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
stonefish_ros2__msg__NEDPose__destroy(stonefish_ros2__msg__NEDPose * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    stonefish_ros2__msg__NEDPose__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
stonefish_ros2__msg__NEDPose__Sequence__init(stonefish_ros2__msg__NEDPose__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stonefish_ros2__msg__NEDPose * data = NULL;

  if (size) {
    data = (stonefish_ros2__msg__NEDPose *)allocator.zero_allocate(size, sizeof(stonefish_ros2__msg__NEDPose), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = stonefish_ros2__msg__NEDPose__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        stonefish_ros2__msg__NEDPose__fini(&data[i - 1]);
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
stonefish_ros2__msg__NEDPose__Sequence__fini(stonefish_ros2__msg__NEDPose__Sequence * array)
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
      stonefish_ros2__msg__NEDPose__fini(&array->data[i]);
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

stonefish_ros2__msg__NEDPose__Sequence *
stonefish_ros2__msg__NEDPose__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stonefish_ros2__msg__NEDPose__Sequence * array = (stonefish_ros2__msg__NEDPose__Sequence *)allocator.allocate(sizeof(stonefish_ros2__msg__NEDPose__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = stonefish_ros2__msg__NEDPose__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
stonefish_ros2__msg__NEDPose__Sequence__destroy(stonefish_ros2__msg__NEDPose__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    stonefish_ros2__msg__NEDPose__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
stonefish_ros2__msg__NEDPose__Sequence__are_equal(const stonefish_ros2__msg__NEDPose__Sequence * lhs, const stonefish_ros2__msg__NEDPose__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!stonefish_ros2__msg__NEDPose__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
stonefish_ros2__msg__NEDPose__Sequence__copy(
  const stonefish_ros2__msg__NEDPose__Sequence * input,
  stonefish_ros2__msg__NEDPose__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(stonefish_ros2__msg__NEDPose);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    stonefish_ros2__msg__NEDPose * data =
      (stonefish_ros2__msg__NEDPose *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!stonefish_ros2__msg__NEDPose__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          stonefish_ros2__msg__NEDPose__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!stonefish_ros2__msg__NEDPose__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
