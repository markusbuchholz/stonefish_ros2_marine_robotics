// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from cola2_msgs:msg/CaptainStateFeedback.idl
// generated code does not contain a copyright notice
#include "cola2_msgs/msg/detail/captain_state_feedback__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `name`
#include "rosidl_runtime_c/string_functions.h"
// Member `keyvalues`
#include "cola2_msgs/msg/detail/key_value__functions.h"

bool
cola2_msgs__msg__CaptainStateFeedback__init(cola2_msgs__msg__CaptainStateFeedback * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    cola2_msgs__msg__CaptainStateFeedback__fini(msg);
    return false;
  }
  // state
  // time_to_finish
  // name
  if (!rosidl_runtime_c__String__init(&msg->name)) {
    cola2_msgs__msg__CaptainStateFeedback__fini(msg);
    return false;
  }
  // id
  // keyvalues
  if (!cola2_msgs__msg__KeyValue__Sequence__init(&msg->keyvalues, 0)) {
    cola2_msgs__msg__CaptainStateFeedback__fini(msg);
    return false;
  }
  return true;
}

void
cola2_msgs__msg__CaptainStateFeedback__fini(cola2_msgs__msg__CaptainStateFeedback * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // state
  // time_to_finish
  // name
  rosidl_runtime_c__String__fini(&msg->name);
  // id
  // keyvalues
  cola2_msgs__msg__KeyValue__Sequence__fini(&msg->keyvalues);
}

bool
cola2_msgs__msg__CaptainStateFeedback__are_equal(const cola2_msgs__msg__CaptainStateFeedback * lhs, const cola2_msgs__msg__CaptainStateFeedback * rhs)
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
  // time_to_finish
  if (lhs->time_to_finish != rhs->time_to_finish) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->name), &(rhs->name)))
  {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // keyvalues
  if (!cola2_msgs__msg__KeyValue__Sequence__are_equal(
      &(lhs->keyvalues), &(rhs->keyvalues)))
  {
    return false;
  }
  return true;
}

bool
cola2_msgs__msg__CaptainStateFeedback__copy(
  const cola2_msgs__msg__CaptainStateFeedback * input,
  cola2_msgs__msg__CaptainStateFeedback * output)
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
  // time_to_finish
  output->time_to_finish = input->time_to_finish;
  // name
  if (!rosidl_runtime_c__String__copy(
      &(input->name), &(output->name)))
  {
    return false;
  }
  // id
  output->id = input->id;
  // keyvalues
  if (!cola2_msgs__msg__KeyValue__Sequence__copy(
      &(input->keyvalues), &(output->keyvalues)))
  {
    return false;
  }
  return true;
}

cola2_msgs__msg__CaptainStateFeedback *
cola2_msgs__msg__CaptainStateFeedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__msg__CaptainStateFeedback * msg = (cola2_msgs__msg__CaptainStateFeedback *)allocator.allocate(sizeof(cola2_msgs__msg__CaptainStateFeedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(cola2_msgs__msg__CaptainStateFeedback));
  bool success = cola2_msgs__msg__CaptainStateFeedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
cola2_msgs__msg__CaptainStateFeedback__destroy(cola2_msgs__msg__CaptainStateFeedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    cola2_msgs__msg__CaptainStateFeedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
cola2_msgs__msg__CaptainStateFeedback__Sequence__init(cola2_msgs__msg__CaptainStateFeedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__msg__CaptainStateFeedback * data = NULL;

  if (size) {
    data = (cola2_msgs__msg__CaptainStateFeedback *)allocator.zero_allocate(size, sizeof(cola2_msgs__msg__CaptainStateFeedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = cola2_msgs__msg__CaptainStateFeedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        cola2_msgs__msg__CaptainStateFeedback__fini(&data[i - 1]);
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
cola2_msgs__msg__CaptainStateFeedback__Sequence__fini(cola2_msgs__msg__CaptainStateFeedback__Sequence * array)
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
      cola2_msgs__msg__CaptainStateFeedback__fini(&array->data[i]);
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

cola2_msgs__msg__CaptainStateFeedback__Sequence *
cola2_msgs__msg__CaptainStateFeedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__msg__CaptainStateFeedback__Sequence * array = (cola2_msgs__msg__CaptainStateFeedback__Sequence *)allocator.allocate(sizeof(cola2_msgs__msg__CaptainStateFeedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = cola2_msgs__msg__CaptainStateFeedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
cola2_msgs__msg__CaptainStateFeedback__Sequence__destroy(cola2_msgs__msg__CaptainStateFeedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    cola2_msgs__msg__CaptainStateFeedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
cola2_msgs__msg__CaptainStateFeedback__Sequence__are_equal(const cola2_msgs__msg__CaptainStateFeedback__Sequence * lhs, const cola2_msgs__msg__CaptainStateFeedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!cola2_msgs__msg__CaptainStateFeedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
cola2_msgs__msg__CaptainStateFeedback__Sequence__copy(
  const cola2_msgs__msg__CaptainStateFeedback__Sequence * input,
  cola2_msgs__msg__CaptainStateFeedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(cola2_msgs__msg__CaptainStateFeedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    cola2_msgs__msg__CaptainStateFeedback * data =
      (cola2_msgs__msg__CaptainStateFeedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!cola2_msgs__msg__CaptainStateFeedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          cola2_msgs__msg__CaptainStateFeedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!cola2_msgs__msg__CaptainStateFeedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
