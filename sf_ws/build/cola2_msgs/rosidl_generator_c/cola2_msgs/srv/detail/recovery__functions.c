// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from cola2_msgs:srv/Recovery.idl
// generated code does not contain a copyright notice
#include "cola2_msgs/srv/detail/recovery__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `requested_action`
#include "cola2_msgs/msg/detail/recovery_action__functions.h"

bool
cola2_msgs__srv__Recovery_Request__init(cola2_msgs__srv__Recovery_Request * msg)
{
  if (!msg) {
    return false;
  }
  // requested_action
  if (!cola2_msgs__msg__RecoveryAction__init(&msg->requested_action)) {
    cola2_msgs__srv__Recovery_Request__fini(msg);
    return false;
  }
  return true;
}

void
cola2_msgs__srv__Recovery_Request__fini(cola2_msgs__srv__Recovery_Request * msg)
{
  if (!msg) {
    return;
  }
  // requested_action
  cola2_msgs__msg__RecoveryAction__fini(&msg->requested_action);
}

bool
cola2_msgs__srv__Recovery_Request__are_equal(const cola2_msgs__srv__Recovery_Request * lhs, const cola2_msgs__srv__Recovery_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // requested_action
  if (!cola2_msgs__msg__RecoveryAction__are_equal(
      &(lhs->requested_action), &(rhs->requested_action)))
  {
    return false;
  }
  return true;
}

bool
cola2_msgs__srv__Recovery_Request__copy(
  const cola2_msgs__srv__Recovery_Request * input,
  cola2_msgs__srv__Recovery_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // requested_action
  if (!cola2_msgs__msg__RecoveryAction__copy(
      &(input->requested_action), &(output->requested_action)))
  {
    return false;
  }
  return true;
}

cola2_msgs__srv__Recovery_Request *
cola2_msgs__srv__Recovery_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__srv__Recovery_Request * msg = (cola2_msgs__srv__Recovery_Request *)allocator.allocate(sizeof(cola2_msgs__srv__Recovery_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(cola2_msgs__srv__Recovery_Request));
  bool success = cola2_msgs__srv__Recovery_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
cola2_msgs__srv__Recovery_Request__destroy(cola2_msgs__srv__Recovery_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    cola2_msgs__srv__Recovery_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
cola2_msgs__srv__Recovery_Request__Sequence__init(cola2_msgs__srv__Recovery_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__srv__Recovery_Request * data = NULL;

  if (size) {
    data = (cola2_msgs__srv__Recovery_Request *)allocator.zero_allocate(size, sizeof(cola2_msgs__srv__Recovery_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = cola2_msgs__srv__Recovery_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        cola2_msgs__srv__Recovery_Request__fini(&data[i - 1]);
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
cola2_msgs__srv__Recovery_Request__Sequence__fini(cola2_msgs__srv__Recovery_Request__Sequence * array)
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
      cola2_msgs__srv__Recovery_Request__fini(&array->data[i]);
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

cola2_msgs__srv__Recovery_Request__Sequence *
cola2_msgs__srv__Recovery_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__srv__Recovery_Request__Sequence * array = (cola2_msgs__srv__Recovery_Request__Sequence *)allocator.allocate(sizeof(cola2_msgs__srv__Recovery_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = cola2_msgs__srv__Recovery_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
cola2_msgs__srv__Recovery_Request__Sequence__destroy(cola2_msgs__srv__Recovery_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    cola2_msgs__srv__Recovery_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
cola2_msgs__srv__Recovery_Request__Sequence__are_equal(const cola2_msgs__srv__Recovery_Request__Sequence * lhs, const cola2_msgs__srv__Recovery_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!cola2_msgs__srv__Recovery_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
cola2_msgs__srv__Recovery_Request__Sequence__copy(
  const cola2_msgs__srv__Recovery_Request__Sequence * input,
  cola2_msgs__srv__Recovery_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(cola2_msgs__srv__Recovery_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    cola2_msgs__srv__Recovery_Request * data =
      (cola2_msgs__srv__Recovery_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!cola2_msgs__srv__Recovery_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          cola2_msgs__srv__Recovery_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!cola2_msgs__srv__Recovery_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
cola2_msgs__srv__Recovery_Response__init(cola2_msgs__srv__Recovery_Response * msg)
{
  if (!msg) {
    return false;
  }
  // attempted
  return true;
}

void
cola2_msgs__srv__Recovery_Response__fini(cola2_msgs__srv__Recovery_Response * msg)
{
  if (!msg) {
    return;
  }
  // attempted
}

bool
cola2_msgs__srv__Recovery_Response__are_equal(const cola2_msgs__srv__Recovery_Response * lhs, const cola2_msgs__srv__Recovery_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // attempted
  if (lhs->attempted != rhs->attempted) {
    return false;
  }
  return true;
}

bool
cola2_msgs__srv__Recovery_Response__copy(
  const cola2_msgs__srv__Recovery_Response * input,
  cola2_msgs__srv__Recovery_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // attempted
  output->attempted = input->attempted;
  return true;
}

cola2_msgs__srv__Recovery_Response *
cola2_msgs__srv__Recovery_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__srv__Recovery_Response * msg = (cola2_msgs__srv__Recovery_Response *)allocator.allocate(sizeof(cola2_msgs__srv__Recovery_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(cola2_msgs__srv__Recovery_Response));
  bool success = cola2_msgs__srv__Recovery_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
cola2_msgs__srv__Recovery_Response__destroy(cola2_msgs__srv__Recovery_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    cola2_msgs__srv__Recovery_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
cola2_msgs__srv__Recovery_Response__Sequence__init(cola2_msgs__srv__Recovery_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__srv__Recovery_Response * data = NULL;

  if (size) {
    data = (cola2_msgs__srv__Recovery_Response *)allocator.zero_allocate(size, sizeof(cola2_msgs__srv__Recovery_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = cola2_msgs__srv__Recovery_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        cola2_msgs__srv__Recovery_Response__fini(&data[i - 1]);
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
cola2_msgs__srv__Recovery_Response__Sequence__fini(cola2_msgs__srv__Recovery_Response__Sequence * array)
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
      cola2_msgs__srv__Recovery_Response__fini(&array->data[i]);
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

cola2_msgs__srv__Recovery_Response__Sequence *
cola2_msgs__srv__Recovery_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__srv__Recovery_Response__Sequence * array = (cola2_msgs__srv__Recovery_Response__Sequence *)allocator.allocate(sizeof(cola2_msgs__srv__Recovery_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = cola2_msgs__srv__Recovery_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
cola2_msgs__srv__Recovery_Response__Sequence__destroy(cola2_msgs__srv__Recovery_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    cola2_msgs__srv__Recovery_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
cola2_msgs__srv__Recovery_Response__Sequence__are_equal(const cola2_msgs__srv__Recovery_Response__Sequence * lhs, const cola2_msgs__srv__Recovery_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!cola2_msgs__srv__Recovery_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
cola2_msgs__srv__Recovery_Response__Sequence__copy(
  const cola2_msgs__srv__Recovery_Response__Sequence * input,
  cola2_msgs__srv__Recovery_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(cola2_msgs__srv__Recovery_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    cola2_msgs__srv__Recovery_Response * data =
      (cola2_msgs__srv__Recovery_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!cola2_msgs__srv__Recovery_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          cola2_msgs__srv__Recovery_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!cola2_msgs__srv__Recovery_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
