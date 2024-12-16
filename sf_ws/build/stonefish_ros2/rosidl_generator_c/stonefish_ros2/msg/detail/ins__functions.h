// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from stonefish_ros2:msg/INS.idl
// generated code does not contain a copyright notice

#ifndef STONEFISH_ROS2__MSG__DETAIL__INS__FUNCTIONS_H_
#define STONEFISH_ROS2__MSG__DETAIL__INS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "stonefish_ros2/msg/rosidl_generator_c__visibility_control.h"

#include "stonefish_ros2/msg/detail/ins__struct.h"

/// Initialize msg/INS message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * stonefish_ros2__msg__INS
 * )) before or use
 * stonefish_ros2__msg__INS__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_stonefish_ros2
bool
stonefish_ros2__msg__INS__init(stonefish_ros2__msg__INS * msg);

/// Finalize msg/INS message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stonefish_ros2
void
stonefish_ros2__msg__INS__fini(stonefish_ros2__msg__INS * msg);

/// Create msg/INS message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * stonefish_ros2__msg__INS__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_stonefish_ros2
stonefish_ros2__msg__INS *
stonefish_ros2__msg__INS__create();

/// Destroy msg/INS message.
/**
 * It calls
 * stonefish_ros2__msg__INS__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stonefish_ros2
void
stonefish_ros2__msg__INS__destroy(stonefish_ros2__msg__INS * msg);

/// Check for msg/INS message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_stonefish_ros2
bool
stonefish_ros2__msg__INS__are_equal(const stonefish_ros2__msg__INS * lhs, const stonefish_ros2__msg__INS * rhs);

/// Copy a msg/INS message.
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
ROSIDL_GENERATOR_C_PUBLIC_stonefish_ros2
bool
stonefish_ros2__msg__INS__copy(
  const stonefish_ros2__msg__INS * input,
  stonefish_ros2__msg__INS * output);

/// Initialize array of msg/INS messages.
/**
 * It allocates the memory for the number of elements and calls
 * stonefish_ros2__msg__INS__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_stonefish_ros2
bool
stonefish_ros2__msg__INS__Sequence__init(stonefish_ros2__msg__INS__Sequence * array, size_t size);

/// Finalize array of msg/INS messages.
/**
 * It calls
 * stonefish_ros2__msg__INS__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stonefish_ros2
void
stonefish_ros2__msg__INS__Sequence__fini(stonefish_ros2__msg__INS__Sequence * array);

/// Create array of msg/INS messages.
/**
 * It allocates the memory for the array and calls
 * stonefish_ros2__msg__INS__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_stonefish_ros2
stonefish_ros2__msg__INS__Sequence *
stonefish_ros2__msg__INS__Sequence__create(size_t size);

/// Destroy array of msg/INS messages.
/**
 * It calls
 * stonefish_ros2__msg__INS__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stonefish_ros2
void
stonefish_ros2__msg__INS__Sequence__destroy(stonefish_ros2__msg__INS__Sequence * array);

/// Check for msg/INS message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_stonefish_ros2
bool
stonefish_ros2__msg__INS__Sequence__are_equal(const stonefish_ros2__msg__INS__Sequence * lhs, const stonefish_ros2__msg__INS__Sequence * rhs);

/// Copy an array of msg/INS messages.
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
ROSIDL_GENERATOR_C_PUBLIC_stonefish_ros2
bool
stonefish_ros2__msg__INS__Sequence__copy(
  const stonefish_ros2__msg__INS__Sequence * input,
  stonefish_ros2__msg__INS__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // STONEFISH_ROS2__MSG__DETAIL__INS__FUNCTIONS_H_
