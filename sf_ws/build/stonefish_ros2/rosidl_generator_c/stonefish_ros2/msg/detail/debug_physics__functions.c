// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from stonefish_ros2:msg/DebugPhysics.idl
// generated code does not contain a copyright notice
#include "stonefish_ros2/msg/detail/debug_physics__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `cog`
// Member `cob`
// Member `inertia`
// Member `damping_coeff`
// Member `skin_friction_coeff`
#include "geometry_msgs/msg/detail/vector3__functions.h"
// Member `velocity`
#include "geometry_msgs/msg/detail/twist__functions.h"
// Member `buoyancy`
// Member `damping`
// Member `skin_friction`
#include "geometry_msgs/msg/detail/wrench__functions.h"

bool
stonefish_ros2__msg__DebugPhysics__init(stonefish_ros2__msg__DebugPhysics * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    stonefish_ros2__msg__DebugPhysics__fini(msg);
    return false;
  }
  // mass
  // volume
  // surface
  // cog
  if (!geometry_msgs__msg__Vector3__init(&msg->cog)) {
    stonefish_ros2__msg__DebugPhysics__fini(msg);
    return false;
  }
  // cob
  if (!geometry_msgs__msg__Vector3__init(&msg->cob)) {
    stonefish_ros2__msg__DebugPhysics__fini(msg);
    return false;
  }
  // inertia
  if (!geometry_msgs__msg__Vector3__init(&msg->inertia)) {
    stonefish_ros2__msg__DebugPhysics__fini(msg);
    return false;
  }
  // damping_coeff
  if (!geometry_msgs__msg__Vector3__init(&msg->damping_coeff)) {
    stonefish_ros2__msg__DebugPhysics__fini(msg);
    return false;
  }
  // skin_friction_coeff
  if (!geometry_msgs__msg__Vector3__init(&msg->skin_friction_coeff)) {
    stonefish_ros2__msg__DebugPhysics__fini(msg);
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Twist__init(&msg->velocity)) {
    stonefish_ros2__msg__DebugPhysics__fini(msg);
    return false;
  }
  // buoyancy
  if (!geometry_msgs__msg__Wrench__init(&msg->buoyancy)) {
    stonefish_ros2__msg__DebugPhysics__fini(msg);
    return false;
  }
  // damping
  if (!geometry_msgs__msg__Wrench__init(&msg->damping)) {
    stonefish_ros2__msg__DebugPhysics__fini(msg);
    return false;
  }
  // skin_friction
  if (!geometry_msgs__msg__Wrench__init(&msg->skin_friction)) {
    stonefish_ros2__msg__DebugPhysics__fini(msg);
    return false;
  }
  // wetted_surface
  // submerged_volume
  return true;
}

void
stonefish_ros2__msg__DebugPhysics__fini(stonefish_ros2__msg__DebugPhysics * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // mass
  // volume
  // surface
  // cog
  geometry_msgs__msg__Vector3__fini(&msg->cog);
  // cob
  geometry_msgs__msg__Vector3__fini(&msg->cob);
  // inertia
  geometry_msgs__msg__Vector3__fini(&msg->inertia);
  // damping_coeff
  geometry_msgs__msg__Vector3__fini(&msg->damping_coeff);
  // skin_friction_coeff
  geometry_msgs__msg__Vector3__fini(&msg->skin_friction_coeff);
  // velocity
  geometry_msgs__msg__Twist__fini(&msg->velocity);
  // buoyancy
  geometry_msgs__msg__Wrench__fini(&msg->buoyancy);
  // damping
  geometry_msgs__msg__Wrench__fini(&msg->damping);
  // skin_friction
  geometry_msgs__msg__Wrench__fini(&msg->skin_friction);
  // wetted_surface
  // submerged_volume
}

bool
stonefish_ros2__msg__DebugPhysics__are_equal(const stonefish_ros2__msg__DebugPhysics * lhs, const stonefish_ros2__msg__DebugPhysics * rhs)
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
  // mass
  if (lhs->mass != rhs->mass) {
    return false;
  }
  // volume
  if (lhs->volume != rhs->volume) {
    return false;
  }
  // surface
  if (lhs->surface != rhs->surface) {
    return false;
  }
  // cog
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->cog), &(rhs->cog)))
  {
    return false;
  }
  // cob
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->cob), &(rhs->cob)))
  {
    return false;
  }
  // inertia
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->inertia), &(rhs->inertia)))
  {
    return false;
  }
  // damping_coeff
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->damping_coeff), &(rhs->damping_coeff)))
  {
    return false;
  }
  // skin_friction_coeff
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->skin_friction_coeff), &(rhs->skin_friction_coeff)))
  {
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Twist__are_equal(
      &(lhs->velocity), &(rhs->velocity)))
  {
    return false;
  }
  // buoyancy
  if (!geometry_msgs__msg__Wrench__are_equal(
      &(lhs->buoyancy), &(rhs->buoyancy)))
  {
    return false;
  }
  // damping
  if (!geometry_msgs__msg__Wrench__are_equal(
      &(lhs->damping), &(rhs->damping)))
  {
    return false;
  }
  // skin_friction
  if (!geometry_msgs__msg__Wrench__are_equal(
      &(lhs->skin_friction), &(rhs->skin_friction)))
  {
    return false;
  }
  // wetted_surface
  if (lhs->wetted_surface != rhs->wetted_surface) {
    return false;
  }
  // submerged_volume
  if (lhs->submerged_volume != rhs->submerged_volume) {
    return false;
  }
  return true;
}

bool
stonefish_ros2__msg__DebugPhysics__copy(
  const stonefish_ros2__msg__DebugPhysics * input,
  stonefish_ros2__msg__DebugPhysics * output)
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
  // mass
  output->mass = input->mass;
  // volume
  output->volume = input->volume;
  // surface
  output->surface = input->surface;
  // cog
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->cog), &(output->cog)))
  {
    return false;
  }
  // cob
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->cob), &(output->cob)))
  {
    return false;
  }
  // inertia
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->inertia), &(output->inertia)))
  {
    return false;
  }
  // damping_coeff
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->damping_coeff), &(output->damping_coeff)))
  {
    return false;
  }
  // skin_friction_coeff
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->skin_friction_coeff), &(output->skin_friction_coeff)))
  {
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Twist__copy(
      &(input->velocity), &(output->velocity)))
  {
    return false;
  }
  // buoyancy
  if (!geometry_msgs__msg__Wrench__copy(
      &(input->buoyancy), &(output->buoyancy)))
  {
    return false;
  }
  // damping
  if (!geometry_msgs__msg__Wrench__copy(
      &(input->damping), &(output->damping)))
  {
    return false;
  }
  // skin_friction
  if (!geometry_msgs__msg__Wrench__copy(
      &(input->skin_friction), &(output->skin_friction)))
  {
    return false;
  }
  // wetted_surface
  output->wetted_surface = input->wetted_surface;
  // submerged_volume
  output->submerged_volume = input->submerged_volume;
  return true;
}

stonefish_ros2__msg__DebugPhysics *
stonefish_ros2__msg__DebugPhysics__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stonefish_ros2__msg__DebugPhysics * msg = (stonefish_ros2__msg__DebugPhysics *)allocator.allocate(sizeof(stonefish_ros2__msg__DebugPhysics), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(stonefish_ros2__msg__DebugPhysics));
  bool success = stonefish_ros2__msg__DebugPhysics__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
stonefish_ros2__msg__DebugPhysics__destroy(stonefish_ros2__msg__DebugPhysics * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    stonefish_ros2__msg__DebugPhysics__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
stonefish_ros2__msg__DebugPhysics__Sequence__init(stonefish_ros2__msg__DebugPhysics__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stonefish_ros2__msg__DebugPhysics * data = NULL;

  if (size) {
    data = (stonefish_ros2__msg__DebugPhysics *)allocator.zero_allocate(size, sizeof(stonefish_ros2__msg__DebugPhysics), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = stonefish_ros2__msg__DebugPhysics__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        stonefish_ros2__msg__DebugPhysics__fini(&data[i - 1]);
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
stonefish_ros2__msg__DebugPhysics__Sequence__fini(stonefish_ros2__msg__DebugPhysics__Sequence * array)
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
      stonefish_ros2__msg__DebugPhysics__fini(&array->data[i]);
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

stonefish_ros2__msg__DebugPhysics__Sequence *
stonefish_ros2__msg__DebugPhysics__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stonefish_ros2__msg__DebugPhysics__Sequence * array = (stonefish_ros2__msg__DebugPhysics__Sequence *)allocator.allocate(sizeof(stonefish_ros2__msg__DebugPhysics__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = stonefish_ros2__msg__DebugPhysics__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
stonefish_ros2__msg__DebugPhysics__Sequence__destroy(stonefish_ros2__msg__DebugPhysics__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    stonefish_ros2__msg__DebugPhysics__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
stonefish_ros2__msg__DebugPhysics__Sequence__are_equal(const stonefish_ros2__msg__DebugPhysics__Sequence * lhs, const stonefish_ros2__msg__DebugPhysics__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!stonefish_ros2__msg__DebugPhysics__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
stonefish_ros2__msg__DebugPhysics__Sequence__copy(
  const stonefish_ros2__msg__DebugPhysics__Sequence * input,
  stonefish_ros2__msg__DebugPhysics__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(stonefish_ros2__msg__DebugPhysics);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    stonefish_ros2__msg__DebugPhysics * data =
      (stonefish_ros2__msg__DebugPhysics *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!stonefish_ros2__msg__DebugPhysics__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          stonefish_ros2__msg__DebugPhysics__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!stonefish_ros2__msg__DebugPhysics__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
