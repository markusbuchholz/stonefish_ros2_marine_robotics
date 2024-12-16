// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from cola2_msgs:msg/SafetySupervisorStatus.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__MSG__DETAIL__SAFETY_SUPERVISOR_STATUS__FUNCTIONS_H_
#define COLA2_MSGS__MSG__DETAIL__SAFETY_SUPERVISOR_STATUS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "cola2_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "cola2_msgs/msg/detail/safety_supervisor_status__struct.h"

/// Initialize msg/SafetySupervisorStatus message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * cola2_msgs__msg__SafetySupervisorStatus
 * )) before or use
 * cola2_msgs__msg__SafetySupervisorStatus__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
bool
cola2_msgs__msg__SafetySupervisorStatus__init(cola2_msgs__msg__SafetySupervisorStatus * msg);

/// Finalize msg/SafetySupervisorStatus message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
void
cola2_msgs__msg__SafetySupervisorStatus__fini(cola2_msgs__msg__SafetySupervisorStatus * msg);

/// Create msg/SafetySupervisorStatus message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * cola2_msgs__msg__SafetySupervisorStatus__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
cola2_msgs__msg__SafetySupervisorStatus *
cola2_msgs__msg__SafetySupervisorStatus__create();

/// Destroy msg/SafetySupervisorStatus message.
/**
 * It calls
 * cola2_msgs__msg__SafetySupervisorStatus__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
void
cola2_msgs__msg__SafetySupervisorStatus__destroy(cola2_msgs__msg__SafetySupervisorStatus * msg);

/// Check for msg/SafetySupervisorStatus message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
bool
cola2_msgs__msg__SafetySupervisorStatus__are_equal(const cola2_msgs__msg__SafetySupervisorStatus * lhs, const cola2_msgs__msg__SafetySupervisorStatus * rhs);

/// Copy a msg/SafetySupervisorStatus message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
bool
cola2_msgs__msg__SafetySupervisorStatus__copy(
  const cola2_msgs__msg__SafetySupervisorStatus * input,
  cola2_msgs__msg__SafetySupervisorStatus * output);

/// Initialize array of msg/SafetySupervisorStatus messages.
/**
 * It allocates the memory for the number of elements and calls
 * cola2_msgs__msg__SafetySupervisorStatus__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
bool
cola2_msgs__msg__SafetySupervisorStatus__Sequence__init(cola2_msgs__msg__SafetySupervisorStatus__Sequence * array, size_t size);

/// Finalize array of msg/SafetySupervisorStatus messages.
/**
 * It calls
 * cola2_msgs__msg__SafetySupervisorStatus__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
void
cola2_msgs__msg__SafetySupervisorStatus__Sequence__fini(cola2_msgs__msg__SafetySupervisorStatus__Sequence * array);

/// Create array of msg/SafetySupervisorStatus messages.
/**
 * It allocates the memory for the array and calls
 * cola2_msgs__msg__SafetySupervisorStatus__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
cola2_msgs__msg__SafetySupervisorStatus__Sequence *
cola2_msgs__msg__SafetySupervisorStatus__Sequence__create(size_t size);

/// Destroy array of msg/SafetySupervisorStatus messages.
/**
 * It calls
 * cola2_msgs__msg__SafetySupervisorStatus__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
void
cola2_msgs__msg__SafetySupervisorStatus__Sequence__destroy(cola2_msgs__msg__SafetySupervisorStatus__Sequence * array);

/// Check for msg/SafetySupervisorStatus message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
bool
cola2_msgs__msg__SafetySupervisorStatus__Sequence__are_equal(const cola2_msgs__msg__SafetySupervisorStatus__Sequence * lhs, const cola2_msgs__msg__SafetySupervisorStatus__Sequence * rhs);

/// Copy an array of msg/SafetySupervisorStatus messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
bool
cola2_msgs__msg__SafetySupervisorStatus__Sequence__copy(
  const cola2_msgs__msg__SafetySupervisorStatus__Sequence * input,
  cola2_msgs__msg__SafetySupervisorStatus__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // COLA2_MSGS__MSG__DETAIL__SAFETY_SUPERVISOR_STATUS__FUNCTIONS_H_
