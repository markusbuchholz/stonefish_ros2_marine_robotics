// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from cola2_msgs:msg/SafetySupervisorStatus.idl
// generated code does not contain a copyright notice
#include "cola2_msgs/msg/detail/safety_supervisor_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `recovery_action`
#include "cola2_msgs/msg/detail/recovery_action__functions.h"

bool
cola2_msgs__msg__SafetySupervisorStatus__init(cola2_msgs__msg__SafetySupervisorStatus * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    cola2_msgs__msg__SafetySupervisorStatus__fini(msg);
    return false;
  }
  // status_code
  // recovery_action
  if (!cola2_msgs__msg__RecoveryAction__init(&msg->recovery_action)) {
    cola2_msgs__msg__SafetySupervisorStatus__fini(msg);
    return false;
  }
  return true;
}

void
cola2_msgs__msg__SafetySupervisorStatus__fini(cola2_msgs__msg__SafetySupervisorStatus * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // status_code
  // recovery_action
  cola2_msgs__msg__RecoveryAction__fini(&msg->recovery_action);
}

bool
cola2_msgs__msg__SafetySupervisorStatus__are_equal(const cola2_msgs__msg__SafetySupervisorStatus * lhs, const cola2_msgs__msg__SafetySupervisorStatus * rhs)
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
  // status_code
  if (lhs->status_code != rhs->status_code) {
    return false;
  }
  // recovery_action
  if (!cola2_msgs__msg__RecoveryAction__are_equal(
      &(lhs->recovery_action), &(rhs->recovery_action)))
  {
    return false;
  }
  return true;
}

bool
cola2_msgs__msg__SafetySupervisorStatus__copy(
  const cola2_msgs__msg__SafetySupervisorStatus * input,
  cola2_msgs__msg__SafetySupervisorStatus * output)
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
  // status_code
  output->status_code = input->status_code;
  // recovery_action
  if (!cola2_msgs__msg__RecoveryAction__copy(
      &(input->recovery_action), &(output->recovery_action)))
  {
    return false;
  }
  return true;
}

cola2_msgs__msg__SafetySupervisorStatus *
cola2_msgs__msg__SafetySupervisorStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__msg__SafetySupervisorStatus * msg = (cola2_msgs__msg__SafetySupervisorStatus *)allocator.allocate(sizeof(cola2_msgs__msg__SafetySupervisorStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(cola2_msgs__msg__SafetySupervisorStatus));
  bool success = cola2_msgs__msg__SafetySupervisorStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
cola2_msgs__msg__SafetySupervisorStatus__destroy(cola2_msgs__msg__SafetySupervisorStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    cola2_msgs__msg__SafetySupervisorStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
cola2_msgs__msg__SafetySupervisorStatus__Sequence__init(cola2_msgs__msg__SafetySupervisorStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__msg__SafetySupervisorStatus * data = NULL;

  if (size) {
    data = (cola2_msgs__msg__SafetySupervisorStatus *)allocator.zero_allocate(size, sizeof(cola2_msgs__msg__SafetySupervisorStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = cola2_msgs__msg__SafetySupervisorStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        cola2_msgs__msg__SafetySupervisorStatus__fini(&data[i - 1]);
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
cola2_msgs__msg__SafetySupervisorStatus__Sequence__fini(cola2_msgs__msg__SafetySupervisorStatus__Sequence * array)
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
      cola2_msgs__msg__SafetySupervisorStatus__fini(&array->data[i]);
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

cola2_msgs__msg__SafetySupervisorStatus__Sequence *
cola2_msgs__msg__SafetySupervisorStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__msg__SafetySupervisorStatus__Sequence * array = (cola2_msgs__msg__SafetySupervisorStatus__Sequence *)allocator.allocate(sizeof(cola2_msgs__msg__SafetySupervisorStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = cola2_msgs__msg__SafetySupervisorStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
cola2_msgs__msg__SafetySupervisorStatus__Sequence__destroy(cola2_msgs__msg__SafetySupervisorStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    cola2_msgs__msg__SafetySupervisorStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
cola2_msgs__msg__SafetySupervisorStatus__Sequence__are_equal(const cola2_msgs__msg__SafetySupervisorStatus__Sequence * lhs, const cola2_msgs__msg__SafetySupervisorStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!cola2_msgs__msg__SafetySupervisorStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
cola2_msgs__msg__SafetySupervisorStatus__Sequence__copy(
  const cola2_msgs__msg__SafetySupervisorStatus__Sequence * input,
  cola2_msgs__msg__SafetySupervisorStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(cola2_msgs__msg__SafetySupervisorStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    cola2_msgs__msg__SafetySupervisorStatus * data =
      (cola2_msgs__msg__SafetySupervisorStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!cola2_msgs__msg__SafetySupervisorStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          cola2_msgs__msg__SafetySupervisorStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!cola2_msgs__msg__SafetySupervisorStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
