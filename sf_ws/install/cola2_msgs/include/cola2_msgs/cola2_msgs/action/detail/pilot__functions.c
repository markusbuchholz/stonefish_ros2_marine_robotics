// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from cola2_msgs:action/Pilot.idl
// generated code does not contain a copyright notice
#include "cola2_msgs/action/detail/pilot__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `goal`
#include "cola2_msgs/msg/detail/goal_descriptor__functions.h"

bool
cola2_msgs__action__Pilot_Goal__init(cola2_msgs__action__Pilot_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // initial_latitude
  // initial_longitude
  // initial_depth
  // final_latitude
  // final_longitude
  // final_depth
  // final_yaw
  // final_altitude
  // heave_mode
  // surge_velocity
  // tolerance_xy
  // timeout
  // controller_type
  // goal
  if (!cola2_msgs__msg__GoalDescriptor__init(&msg->goal)) {
    cola2_msgs__action__Pilot_Goal__fini(msg);
    return false;
  }
  return true;
}

void
cola2_msgs__action__Pilot_Goal__fini(cola2_msgs__action__Pilot_Goal * msg)
{
  if (!msg) {
    return;
  }
  // initial_latitude
  // initial_longitude
  // initial_depth
  // final_latitude
  // final_longitude
  // final_depth
  // final_yaw
  // final_altitude
  // heave_mode
  // surge_velocity
  // tolerance_xy
  // timeout
  // controller_type
  // goal
  cola2_msgs__msg__GoalDescriptor__fini(&msg->goal);
}

bool
cola2_msgs__action__Pilot_Goal__are_equal(const cola2_msgs__action__Pilot_Goal * lhs, const cola2_msgs__action__Pilot_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // initial_latitude
  if (lhs->initial_latitude != rhs->initial_latitude) {
    return false;
  }
  // initial_longitude
  if (lhs->initial_longitude != rhs->initial_longitude) {
    return false;
  }
  // initial_depth
  if (lhs->initial_depth != rhs->initial_depth) {
    return false;
  }
  // final_latitude
  if (lhs->final_latitude != rhs->final_latitude) {
    return false;
  }
  // final_longitude
  if (lhs->final_longitude != rhs->final_longitude) {
    return false;
  }
  // final_depth
  if (lhs->final_depth != rhs->final_depth) {
    return false;
  }
  // final_yaw
  if (lhs->final_yaw != rhs->final_yaw) {
    return false;
  }
  // final_altitude
  if (lhs->final_altitude != rhs->final_altitude) {
    return false;
  }
  // heave_mode
  if (lhs->heave_mode != rhs->heave_mode) {
    return false;
  }
  // surge_velocity
  if (lhs->surge_velocity != rhs->surge_velocity) {
    return false;
  }
  // tolerance_xy
  if (lhs->tolerance_xy != rhs->tolerance_xy) {
    return false;
  }
  // timeout
  if (lhs->timeout != rhs->timeout) {
    return false;
  }
  // controller_type
  if (lhs->controller_type != rhs->controller_type) {
    return false;
  }
  // goal
  if (!cola2_msgs__msg__GoalDescriptor__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
cola2_msgs__action__Pilot_Goal__copy(
  const cola2_msgs__action__Pilot_Goal * input,
  cola2_msgs__action__Pilot_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // initial_latitude
  output->initial_latitude = input->initial_latitude;
  // initial_longitude
  output->initial_longitude = input->initial_longitude;
  // initial_depth
  output->initial_depth = input->initial_depth;
  // final_latitude
  output->final_latitude = input->final_latitude;
  // final_longitude
  output->final_longitude = input->final_longitude;
  // final_depth
  output->final_depth = input->final_depth;
  // final_yaw
  output->final_yaw = input->final_yaw;
  // final_altitude
  output->final_altitude = input->final_altitude;
  // heave_mode
  output->heave_mode = input->heave_mode;
  // surge_velocity
  output->surge_velocity = input->surge_velocity;
  // tolerance_xy
  output->tolerance_xy = input->tolerance_xy;
  // timeout
  output->timeout = input->timeout;
  // controller_type
  output->controller_type = input->controller_type;
  // goal
  if (!cola2_msgs__msg__GoalDescriptor__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

cola2_msgs__action__Pilot_Goal *
cola2_msgs__action__Pilot_Goal__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__action__Pilot_Goal * msg = (cola2_msgs__action__Pilot_Goal *)allocator.allocate(sizeof(cola2_msgs__action__Pilot_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(cola2_msgs__action__Pilot_Goal));
  bool success = cola2_msgs__action__Pilot_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
cola2_msgs__action__Pilot_Goal__destroy(cola2_msgs__action__Pilot_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    cola2_msgs__action__Pilot_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
cola2_msgs__action__Pilot_Goal__Sequence__init(cola2_msgs__action__Pilot_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__action__Pilot_Goal * data = NULL;

  if (size) {
    data = (cola2_msgs__action__Pilot_Goal *)allocator.zero_allocate(size, sizeof(cola2_msgs__action__Pilot_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = cola2_msgs__action__Pilot_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        cola2_msgs__action__Pilot_Goal__fini(&data[i - 1]);
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
cola2_msgs__action__Pilot_Goal__Sequence__fini(cola2_msgs__action__Pilot_Goal__Sequence * array)
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
      cola2_msgs__action__Pilot_Goal__fini(&array->data[i]);
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

cola2_msgs__action__Pilot_Goal__Sequence *
cola2_msgs__action__Pilot_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__action__Pilot_Goal__Sequence * array = (cola2_msgs__action__Pilot_Goal__Sequence *)allocator.allocate(sizeof(cola2_msgs__action__Pilot_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = cola2_msgs__action__Pilot_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
cola2_msgs__action__Pilot_Goal__Sequence__destroy(cola2_msgs__action__Pilot_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    cola2_msgs__action__Pilot_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
cola2_msgs__action__Pilot_Goal__Sequence__are_equal(const cola2_msgs__action__Pilot_Goal__Sequence * lhs, const cola2_msgs__action__Pilot_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!cola2_msgs__action__Pilot_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
cola2_msgs__action__Pilot_Goal__Sequence__copy(
  const cola2_msgs__action__Pilot_Goal__Sequence * input,
  cola2_msgs__action__Pilot_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(cola2_msgs__action__Pilot_Goal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    cola2_msgs__action__Pilot_Goal * data =
      (cola2_msgs__action__Pilot_Goal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!cola2_msgs__action__Pilot_Goal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          cola2_msgs__action__Pilot_Goal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!cola2_msgs__action__Pilot_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
cola2_msgs__action__Pilot_Result__init(cola2_msgs__action__Pilot_Result * msg)
{
  if (!msg) {
    return false;
  }
  // state
  return true;
}

void
cola2_msgs__action__Pilot_Result__fini(cola2_msgs__action__Pilot_Result * msg)
{
  if (!msg) {
    return;
  }
  // state
}

bool
cola2_msgs__action__Pilot_Result__are_equal(const cola2_msgs__action__Pilot_Result * lhs, const cola2_msgs__action__Pilot_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // state
  if (lhs->state != rhs->state) {
    return false;
  }
  return true;
}

bool
cola2_msgs__action__Pilot_Result__copy(
  const cola2_msgs__action__Pilot_Result * input,
  cola2_msgs__action__Pilot_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // state
  output->state = input->state;
  return true;
}

cola2_msgs__action__Pilot_Result *
cola2_msgs__action__Pilot_Result__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__action__Pilot_Result * msg = (cola2_msgs__action__Pilot_Result *)allocator.allocate(sizeof(cola2_msgs__action__Pilot_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(cola2_msgs__action__Pilot_Result));
  bool success = cola2_msgs__action__Pilot_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
cola2_msgs__action__Pilot_Result__destroy(cola2_msgs__action__Pilot_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    cola2_msgs__action__Pilot_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
cola2_msgs__action__Pilot_Result__Sequence__init(cola2_msgs__action__Pilot_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__action__Pilot_Result * data = NULL;

  if (size) {
    data = (cola2_msgs__action__Pilot_Result *)allocator.zero_allocate(size, sizeof(cola2_msgs__action__Pilot_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = cola2_msgs__action__Pilot_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        cola2_msgs__action__Pilot_Result__fini(&data[i - 1]);
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
cola2_msgs__action__Pilot_Result__Sequence__fini(cola2_msgs__action__Pilot_Result__Sequence * array)
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
      cola2_msgs__action__Pilot_Result__fini(&array->data[i]);
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

cola2_msgs__action__Pilot_Result__Sequence *
cola2_msgs__action__Pilot_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__action__Pilot_Result__Sequence * array = (cola2_msgs__action__Pilot_Result__Sequence *)allocator.allocate(sizeof(cola2_msgs__action__Pilot_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = cola2_msgs__action__Pilot_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
cola2_msgs__action__Pilot_Result__Sequence__destroy(cola2_msgs__action__Pilot_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    cola2_msgs__action__Pilot_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
cola2_msgs__action__Pilot_Result__Sequence__are_equal(const cola2_msgs__action__Pilot_Result__Sequence * lhs, const cola2_msgs__action__Pilot_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!cola2_msgs__action__Pilot_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
cola2_msgs__action__Pilot_Result__Sequence__copy(
  const cola2_msgs__action__Pilot_Result__Sequence * input,
  cola2_msgs__action__Pilot_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(cola2_msgs__action__Pilot_Result);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    cola2_msgs__action__Pilot_Result * data =
      (cola2_msgs__action__Pilot_Result *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!cola2_msgs__action__Pilot_Result__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          cola2_msgs__action__Pilot_Result__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!cola2_msgs__action__Pilot_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
cola2_msgs__action__Pilot_Feedback__init(cola2_msgs__action__Pilot_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // distance_to_end
  // cross_track_error
  return true;
}

void
cola2_msgs__action__Pilot_Feedback__fini(cola2_msgs__action__Pilot_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // distance_to_end
  // cross_track_error
}

bool
cola2_msgs__action__Pilot_Feedback__are_equal(const cola2_msgs__action__Pilot_Feedback * lhs, const cola2_msgs__action__Pilot_Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // distance_to_end
  if (lhs->distance_to_end != rhs->distance_to_end) {
    return false;
  }
  // cross_track_error
  if (lhs->cross_track_error != rhs->cross_track_error) {
    return false;
  }
  return true;
}

bool
cola2_msgs__action__Pilot_Feedback__copy(
  const cola2_msgs__action__Pilot_Feedback * input,
  cola2_msgs__action__Pilot_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // distance_to_end
  output->distance_to_end = input->distance_to_end;
  // cross_track_error
  output->cross_track_error = input->cross_track_error;
  return true;
}

cola2_msgs__action__Pilot_Feedback *
cola2_msgs__action__Pilot_Feedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__action__Pilot_Feedback * msg = (cola2_msgs__action__Pilot_Feedback *)allocator.allocate(sizeof(cola2_msgs__action__Pilot_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(cola2_msgs__action__Pilot_Feedback));
  bool success = cola2_msgs__action__Pilot_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
cola2_msgs__action__Pilot_Feedback__destroy(cola2_msgs__action__Pilot_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    cola2_msgs__action__Pilot_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
cola2_msgs__action__Pilot_Feedback__Sequence__init(cola2_msgs__action__Pilot_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__action__Pilot_Feedback * data = NULL;

  if (size) {
    data = (cola2_msgs__action__Pilot_Feedback *)allocator.zero_allocate(size, sizeof(cola2_msgs__action__Pilot_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = cola2_msgs__action__Pilot_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        cola2_msgs__action__Pilot_Feedback__fini(&data[i - 1]);
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
cola2_msgs__action__Pilot_Feedback__Sequence__fini(cola2_msgs__action__Pilot_Feedback__Sequence * array)
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
      cola2_msgs__action__Pilot_Feedback__fini(&array->data[i]);
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

cola2_msgs__action__Pilot_Feedback__Sequence *
cola2_msgs__action__Pilot_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__action__Pilot_Feedback__Sequence * array = (cola2_msgs__action__Pilot_Feedback__Sequence *)allocator.allocate(sizeof(cola2_msgs__action__Pilot_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = cola2_msgs__action__Pilot_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
cola2_msgs__action__Pilot_Feedback__Sequence__destroy(cola2_msgs__action__Pilot_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    cola2_msgs__action__Pilot_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
cola2_msgs__action__Pilot_Feedback__Sequence__are_equal(const cola2_msgs__action__Pilot_Feedback__Sequence * lhs, const cola2_msgs__action__Pilot_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!cola2_msgs__action__Pilot_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
cola2_msgs__action__Pilot_Feedback__Sequence__copy(
  const cola2_msgs__action__Pilot_Feedback__Sequence * input,
  cola2_msgs__action__Pilot_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(cola2_msgs__action__Pilot_Feedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    cola2_msgs__action__Pilot_Feedback * data =
      (cola2_msgs__action__Pilot_Feedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!cola2_msgs__action__Pilot_Feedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          cola2_msgs__action__Pilot_Feedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!cola2_msgs__action__Pilot_Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "cola2_msgs/action/detail/pilot__functions.h"

bool
cola2_msgs__action__Pilot_SendGoal_Request__init(cola2_msgs__action__Pilot_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    cola2_msgs__action__Pilot_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!cola2_msgs__action__Pilot_Goal__init(&msg->goal)) {
    cola2_msgs__action__Pilot_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
cola2_msgs__action__Pilot_SendGoal_Request__fini(cola2_msgs__action__Pilot_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  cola2_msgs__action__Pilot_Goal__fini(&msg->goal);
}

bool
cola2_msgs__action__Pilot_SendGoal_Request__are_equal(const cola2_msgs__action__Pilot_SendGoal_Request * lhs, const cola2_msgs__action__Pilot_SendGoal_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // goal
  if (!cola2_msgs__action__Pilot_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
cola2_msgs__action__Pilot_SendGoal_Request__copy(
  const cola2_msgs__action__Pilot_SendGoal_Request * input,
  cola2_msgs__action__Pilot_SendGoal_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // goal
  if (!cola2_msgs__action__Pilot_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

cola2_msgs__action__Pilot_SendGoal_Request *
cola2_msgs__action__Pilot_SendGoal_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__action__Pilot_SendGoal_Request * msg = (cola2_msgs__action__Pilot_SendGoal_Request *)allocator.allocate(sizeof(cola2_msgs__action__Pilot_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(cola2_msgs__action__Pilot_SendGoal_Request));
  bool success = cola2_msgs__action__Pilot_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
cola2_msgs__action__Pilot_SendGoal_Request__destroy(cola2_msgs__action__Pilot_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    cola2_msgs__action__Pilot_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
cola2_msgs__action__Pilot_SendGoal_Request__Sequence__init(cola2_msgs__action__Pilot_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__action__Pilot_SendGoal_Request * data = NULL;

  if (size) {
    data = (cola2_msgs__action__Pilot_SendGoal_Request *)allocator.zero_allocate(size, sizeof(cola2_msgs__action__Pilot_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = cola2_msgs__action__Pilot_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        cola2_msgs__action__Pilot_SendGoal_Request__fini(&data[i - 1]);
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
cola2_msgs__action__Pilot_SendGoal_Request__Sequence__fini(cola2_msgs__action__Pilot_SendGoal_Request__Sequence * array)
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
      cola2_msgs__action__Pilot_SendGoal_Request__fini(&array->data[i]);
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

cola2_msgs__action__Pilot_SendGoal_Request__Sequence *
cola2_msgs__action__Pilot_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__action__Pilot_SendGoal_Request__Sequence * array = (cola2_msgs__action__Pilot_SendGoal_Request__Sequence *)allocator.allocate(sizeof(cola2_msgs__action__Pilot_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = cola2_msgs__action__Pilot_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
cola2_msgs__action__Pilot_SendGoal_Request__Sequence__destroy(cola2_msgs__action__Pilot_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    cola2_msgs__action__Pilot_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
cola2_msgs__action__Pilot_SendGoal_Request__Sequence__are_equal(const cola2_msgs__action__Pilot_SendGoal_Request__Sequence * lhs, const cola2_msgs__action__Pilot_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!cola2_msgs__action__Pilot_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
cola2_msgs__action__Pilot_SendGoal_Request__Sequence__copy(
  const cola2_msgs__action__Pilot_SendGoal_Request__Sequence * input,
  cola2_msgs__action__Pilot_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(cola2_msgs__action__Pilot_SendGoal_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    cola2_msgs__action__Pilot_SendGoal_Request * data =
      (cola2_msgs__action__Pilot_SendGoal_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!cola2_msgs__action__Pilot_SendGoal_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          cola2_msgs__action__Pilot_SendGoal_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!cola2_msgs__action__Pilot_SendGoal_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
cola2_msgs__action__Pilot_SendGoal_Response__init(cola2_msgs__action__Pilot_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    cola2_msgs__action__Pilot_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
cola2_msgs__action__Pilot_SendGoal_Response__fini(cola2_msgs__action__Pilot_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
cola2_msgs__action__Pilot_SendGoal_Response__are_equal(const cola2_msgs__action__Pilot_SendGoal_Response * lhs, const cola2_msgs__action__Pilot_SendGoal_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
cola2_msgs__action__Pilot_SendGoal_Response__copy(
  const cola2_msgs__action__Pilot_SendGoal_Response * input,
  cola2_msgs__action__Pilot_SendGoal_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

cola2_msgs__action__Pilot_SendGoal_Response *
cola2_msgs__action__Pilot_SendGoal_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__action__Pilot_SendGoal_Response * msg = (cola2_msgs__action__Pilot_SendGoal_Response *)allocator.allocate(sizeof(cola2_msgs__action__Pilot_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(cola2_msgs__action__Pilot_SendGoal_Response));
  bool success = cola2_msgs__action__Pilot_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
cola2_msgs__action__Pilot_SendGoal_Response__destroy(cola2_msgs__action__Pilot_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    cola2_msgs__action__Pilot_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
cola2_msgs__action__Pilot_SendGoal_Response__Sequence__init(cola2_msgs__action__Pilot_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__action__Pilot_SendGoal_Response * data = NULL;

  if (size) {
    data = (cola2_msgs__action__Pilot_SendGoal_Response *)allocator.zero_allocate(size, sizeof(cola2_msgs__action__Pilot_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = cola2_msgs__action__Pilot_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        cola2_msgs__action__Pilot_SendGoal_Response__fini(&data[i - 1]);
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
cola2_msgs__action__Pilot_SendGoal_Response__Sequence__fini(cola2_msgs__action__Pilot_SendGoal_Response__Sequence * array)
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
      cola2_msgs__action__Pilot_SendGoal_Response__fini(&array->data[i]);
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

cola2_msgs__action__Pilot_SendGoal_Response__Sequence *
cola2_msgs__action__Pilot_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__action__Pilot_SendGoal_Response__Sequence * array = (cola2_msgs__action__Pilot_SendGoal_Response__Sequence *)allocator.allocate(sizeof(cola2_msgs__action__Pilot_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = cola2_msgs__action__Pilot_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
cola2_msgs__action__Pilot_SendGoal_Response__Sequence__destroy(cola2_msgs__action__Pilot_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    cola2_msgs__action__Pilot_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
cola2_msgs__action__Pilot_SendGoal_Response__Sequence__are_equal(const cola2_msgs__action__Pilot_SendGoal_Response__Sequence * lhs, const cola2_msgs__action__Pilot_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!cola2_msgs__action__Pilot_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
cola2_msgs__action__Pilot_SendGoal_Response__Sequence__copy(
  const cola2_msgs__action__Pilot_SendGoal_Response__Sequence * input,
  cola2_msgs__action__Pilot_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(cola2_msgs__action__Pilot_SendGoal_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    cola2_msgs__action__Pilot_SendGoal_Response * data =
      (cola2_msgs__action__Pilot_SendGoal_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!cola2_msgs__action__Pilot_SendGoal_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          cola2_msgs__action__Pilot_SendGoal_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!cola2_msgs__action__Pilot_SendGoal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
cola2_msgs__action__Pilot_GetResult_Request__init(cola2_msgs__action__Pilot_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    cola2_msgs__action__Pilot_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
cola2_msgs__action__Pilot_GetResult_Request__fini(cola2_msgs__action__Pilot_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
cola2_msgs__action__Pilot_GetResult_Request__are_equal(const cola2_msgs__action__Pilot_GetResult_Request * lhs, const cola2_msgs__action__Pilot_GetResult_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  return true;
}

bool
cola2_msgs__action__Pilot_GetResult_Request__copy(
  const cola2_msgs__action__Pilot_GetResult_Request * input,
  cola2_msgs__action__Pilot_GetResult_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  return true;
}

cola2_msgs__action__Pilot_GetResult_Request *
cola2_msgs__action__Pilot_GetResult_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__action__Pilot_GetResult_Request * msg = (cola2_msgs__action__Pilot_GetResult_Request *)allocator.allocate(sizeof(cola2_msgs__action__Pilot_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(cola2_msgs__action__Pilot_GetResult_Request));
  bool success = cola2_msgs__action__Pilot_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
cola2_msgs__action__Pilot_GetResult_Request__destroy(cola2_msgs__action__Pilot_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    cola2_msgs__action__Pilot_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
cola2_msgs__action__Pilot_GetResult_Request__Sequence__init(cola2_msgs__action__Pilot_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__action__Pilot_GetResult_Request * data = NULL;

  if (size) {
    data = (cola2_msgs__action__Pilot_GetResult_Request *)allocator.zero_allocate(size, sizeof(cola2_msgs__action__Pilot_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = cola2_msgs__action__Pilot_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        cola2_msgs__action__Pilot_GetResult_Request__fini(&data[i - 1]);
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
cola2_msgs__action__Pilot_GetResult_Request__Sequence__fini(cola2_msgs__action__Pilot_GetResult_Request__Sequence * array)
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
      cola2_msgs__action__Pilot_GetResult_Request__fini(&array->data[i]);
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

cola2_msgs__action__Pilot_GetResult_Request__Sequence *
cola2_msgs__action__Pilot_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__action__Pilot_GetResult_Request__Sequence * array = (cola2_msgs__action__Pilot_GetResult_Request__Sequence *)allocator.allocate(sizeof(cola2_msgs__action__Pilot_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = cola2_msgs__action__Pilot_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
cola2_msgs__action__Pilot_GetResult_Request__Sequence__destroy(cola2_msgs__action__Pilot_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    cola2_msgs__action__Pilot_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
cola2_msgs__action__Pilot_GetResult_Request__Sequence__are_equal(const cola2_msgs__action__Pilot_GetResult_Request__Sequence * lhs, const cola2_msgs__action__Pilot_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!cola2_msgs__action__Pilot_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
cola2_msgs__action__Pilot_GetResult_Request__Sequence__copy(
  const cola2_msgs__action__Pilot_GetResult_Request__Sequence * input,
  cola2_msgs__action__Pilot_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(cola2_msgs__action__Pilot_GetResult_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    cola2_msgs__action__Pilot_GetResult_Request * data =
      (cola2_msgs__action__Pilot_GetResult_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!cola2_msgs__action__Pilot_GetResult_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          cola2_msgs__action__Pilot_GetResult_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!cola2_msgs__action__Pilot_GetResult_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result`
// already included above
// #include "cola2_msgs/action/detail/pilot__functions.h"

bool
cola2_msgs__action__Pilot_GetResult_Response__init(cola2_msgs__action__Pilot_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!cola2_msgs__action__Pilot_Result__init(&msg->result)) {
    cola2_msgs__action__Pilot_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
cola2_msgs__action__Pilot_GetResult_Response__fini(cola2_msgs__action__Pilot_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  cola2_msgs__action__Pilot_Result__fini(&msg->result);
}

bool
cola2_msgs__action__Pilot_GetResult_Response__are_equal(const cola2_msgs__action__Pilot_GetResult_Response * lhs, const cola2_msgs__action__Pilot_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!cola2_msgs__action__Pilot_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
cola2_msgs__action__Pilot_GetResult_Response__copy(
  const cola2_msgs__action__Pilot_GetResult_Response * input,
  cola2_msgs__action__Pilot_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!cola2_msgs__action__Pilot_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

cola2_msgs__action__Pilot_GetResult_Response *
cola2_msgs__action__Pilot_GetResult_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__action__Pilot_GetResult_Response * msg = (cola2_msgs__action__Pilot_GetResult_Response *)allocator.allocate(sizeof(cola2_msgs__action__Pilot_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(cola2_msgs__action__Pilot_GetResult_Response));
  bool success = cola2_msgs__action__Pilot_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
cola2_msgs__action__Pilot_GetResult_Response__destroy(cola2_msgs__action__Pilot_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    cola2_msgs__action__Pilot_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
cola2_msgs__action__Pilot_GetResult_Response__Sequence__init(cola2_msgs__action__Pilot_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__action__Pilot_GetResult_Response * data = NULL;

  if (size) {
    data = (cola2_msgs__action__Pilot_GetResult_Response *)allocator.zero_allocate(size, sizeof(cola2_msgs__action__Pilot_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = cola2_msgs__action__Pilot_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        cola2_msgs__action__Pilot_GetResult_Response__fini(&data[i - 1]);
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
cola2_msgs__action__Pilot_GetResult_Response__Sequence__fini(cola2_msgs__action__Pilot_GetResult_Response__Sequence * array)
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
      cola2_msgs__action__Pilot_GetResult_Response__fini(&array->data[i]);
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

cola2_msgs__action__Pilot_GetResult_Response__Sequence *
cola2_msgs__action__Pilot_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__action__Pilot_GetResult_Response__Sequence * array = (cola2_msgs__action__Pilot_GetResult_Response__Sequence *)allocator.allocate(sizeof(cola2_msgs__action__Pilot_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = cola2_msgs__action__Pilot_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
cola2_msgs__action__Pilot_GetResult_Response__Sequence__destroy(cola2_msgs__action__Pilot_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    cola2_msgs__action__Pilot_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
cola2_msgs__action__Pilot_GetResult_Response__Sequence__are_equal(const cola2_msgs__action__Pilot_GetResult_Response__Sequence * lhs, const cola2_msgs__action__Pilot_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!cola2_msgs__action__Pilot_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
cola2_msgs__action__Pilot_GetResult_Response__Sequence__copy(
  const cola2_msgs__action__Pilot_GetResult_Response__Sequence * input,
  cola2_msgs__action__Pilot_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(cola2_msgs__action__Pilot_GetResult_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    cola2_msgs__action__Pilot_GetResult_Response * data =
      (cola2_msgs__action__Pilot_GetResult_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!cola2_msgs__action__Pilot_GetResult_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          cola2_msgs__action__Pilot_GetResult_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!cola2_msgs__action__Pilot_GetResult_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "cola2_msgs/action/detail/pilot__functions.h"

bool
cola2_msgs__action__Pilot_FeedbackMessage__init(cola2_msgs__action__Pilot_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    cola2_msgs__action__Pilot_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!cola2_msgs__action__Pilot_Feedback__init(&msg->feedback)) {
    cola2_msgs__action__Pilot_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
cola2_msgs__action__Pilot_FeedbackMessage__fini(cola2_msgs__action__Pilot_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  cola2_msgs__action__Pilot_Feedback__fini(&msg->feedback);
}

bool
cola2_msgs__action__Pilot_FeedbackMessage__are_equal(const cola2_msgs__action__Pilot_FeedbackMessage * lhs, const cola2_msgs__action__Pilot_FeedbackMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // feedback
  if (!cola2_msgs__action__Pilot_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
cola2_msgs__action__Pilot_FeedbackMessage__copy(
  const cola2_msgs__action__Pilot_FeedbackMessage * input,
  cola2_msgs__action__Pilot_FeedbackMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // feedback
  if (!cola2_msgs__action__Pilot_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

cola2_msgs__action__Pilot_FeedbackMessage *
cola2_msgs__action__Pilot_FeedbackMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__action__Pilot_FeedbackMessage * msg = (cola2_msgs__action__Pilot_FeedbackMessage *)allocator.allocate(sizeof(cola2_msgs__action__Pilot_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(cola2_msgs__action__Pilot_FeedbackMessage));
  bool success = cola2_msgs__action__Pilot_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
cola2_msgs__action__Pilot_FeedbackMessage__destroy(cola2_msgs__action__Pilot_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    cola2_msgs__action__Pilot_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
cola2_msgs__action__Pilot_FeedbackMessage__Sequence__init(cola2_msgs__action__Pilot_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__action__Pilot_FeedbackMessage * data = NULL;

  if (size) {
    data = (cola2_msgs__action__Pilot_FeedbackMessage *)allocator.zero_allocate(size, sizeof(cola2_msgs__action__Pilot_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = cola2_msgs__action__Pilot_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        cola2_msgs__action__Pilot_FeedbackMessage__fini(&data[i - 1]);
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
cola2_msgs__action__Pilot_FeedbackMessage__Sequence__fini(cola2_msgs__action__Pilot_FeedbackMessage__Sequence * array)
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
      cola2_msgs__action__Pilot_FeedbackMessage__fini(&array->data[i]);
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

cola2_msgs__action__Pilot_FeedbackMessage__Sequence *
cola2_msgs__action__Pilot_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cola2_msgs__action__Pilot_FeedbackMessage__Sequence * array = (cola2_msgs__action__Pilot_FeedbackMessage__Sequence *)allocator.allocate(sizeof(cola2_msgs__action__Pilot_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = cola2_msgs__action__Pilot_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
cola2_msgs__action__Pilot_FeedbackMessage__Sequence__destroy(cola2_msgs__action__Pilot_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    cola2_msgs__action__Pilot_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
cola2_msgs__action__Pilot_FeedbackMessage__Sequence__are_equal(const cola2_msgs__action__Pilot_FeedbackMessage__Sequence * lhs, const cola2_msgs__action__Pilot_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!cola2_msgs__action__Pilot_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
cola2_msgs__action__Pilot_FeedbackMessage__Sequence__copy(
  const cola2_msgs__action__Pilot_FeedbackMessage__Sequence * input,
  cola2_msgs__action__Pilot_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(cola2_msgs__action__Pilot_FeedbackMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    cola2_msgs__action__Pilot_FeedbackMessage * data =
      (cola2_msgs__action__Pilot_FeedbackMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!cola2_msgs__action__Pilot_FeedbackMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          cola2_msgs__action__Pilot_FeedbackMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!cola2_msgs__action__Pilot_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
