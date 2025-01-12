// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from cola2_msgs:action/Pilot.idl
// generated code does not contain a copyright notice

#ifndef COLA2_MSGS__ACTION__DETAIL__PILOT__FUNCTIONS_H_
#define COLA2_MSGS__ACTION__DETAIL__PILOT__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "cola2_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "cola2_msgs/action/detail/pilot__struct.h"

/// Initialize action/Pilot message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * cola2_msgs__action__Pilot_Goal
 * )) before or use
 * cola2_msgs__action__Pilot_Goal__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
bool
cola2_msgs__action__Pilot_Goal__init(cola2_msgs__action__Pilot_Goal * msg);

/// Finalize action/Pilot message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
void
cola2_msgs__action__Pilot_Goal__fini(cola2_msgs__action__Pilot_Goal * msg);

/// Create action/Pilot message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * cola2_msgs__action__Pilot_Goal__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
cola2_msgs__action__Pilot_Goal *
cola2_msgs__action__Pilot_Goal__create();

/// Destroy action/Pilot message.
/**
 * It calls
 * cola2_msgs__action__Pilot_Goal__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
void
cola2_msgs__action__Pilot_Goal__destroy(cola2_msgs__action__Pilot_Goal * msg);

/// Check for action/Pilot message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
bool
cola2_msgs__action__Pilot_Goal__are_equal(const cola2_msgs__action__Pilot_Goal * lhs, const cola2_msgs__action__Pilot_Goal * rhs);

/// Copy a action/Pilot message.
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
cola2_msgs__action__Pilot_Goal__copy(
  const cola2_msgs__action__Pilot_Goal * input,
  cola2_msgs__action__Pilot_Goal * output);

/// Initialize array of action/Pilot messages.
/**
 * It allocates the memory for the number of elements and calls
 * cola2_msgs__action__Pilot_Goal__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
bool
cola2_msgs__action__Pilot_Goal__Sequence__init(cola2_msgs__action__Pilot_Goal__Sequence * array, size_t size);

/// Finalize array of action/Pilot messages.
/**
 * It calls
 * cola2_msgs__action__Pilot_Goal__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
void
cola2_msgs__action__Pilot_Goal__Sequence__fini(cola2_msgs__action__Pilot_Goal__Sequence * array);

/// Create array of action/Pilot messages.
/**
 * It allocates the memory for the array and calls
 * cola2_msgs__action__Pilot_Goal__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
cola2_msgs__action__Pilot_Goal__Sequence *
cola2_msgs__action__Pilot_Goal__Sequence__create(size_t size);

/// Destroy array of action/Pilot messages.
/**
 * It calls
 * cola2_msgs__action__Pilot_Goal__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
void
cola2_msgs__action__Pilot_Goal__Sequence__destroy(cola2_msgs__action__Pilot_Goal__Sequence * array);

/// Check for action/Pilot message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
bool
cola2_msgs__action__Pilot_Goal__Sequence__are_equal(const cola2_msgs__action__Pilot_Goal__Sequence * lhs, const cola2_msgs__action__Pilot_Goal__Sequence * rhs);

/// Copy an array of action/Pilot messages.
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
cola2_msgs__action__Pilot_Goal__Sequence__copy(
  const cola2_msgs__action__Pilot_Goal__Sequence * input,
  cola2_msgs__action__Pilot_Goal__Sequence * output);

/// Initialize action/Pilot message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * cola2_msgs__action__Pilot_Result
 * )) before or use
 * cola2_msgs__action__Pilot_Result__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
bool
cola2_msgs__action__Pilot_Result__init(cola2_msgs__action__Pilot_Result * msg);

/// Finalize action/Pilot message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
void
cola2_msgs__action__Pilot_Result__fini(cola2_msgs__action__Pilot_Result * msg);

/// Create action/Pilot message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * cola2_msgs__action__Pilot_Result__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
cola2_msgs__action__Pilot_Result *
cola2_msgs__action__Pilot_Result__create();

/// Destroy action/Pilot message.
/**
 * It calls
 * cola2_msgs__action__Pilot_Result__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
void
cola2_msgs__action__Pilot_Result__destroy(cola2_msgs__action__Pilot_Result * msg);

/// Check for action/Pilot message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
bool
cola2_msgs__action__Pilot_Result__are_equal(const cola2_msgs__action__Pilot_Result * lhs, const cola2_msgs__action__Pilot_Result * rhs);

/// Copy a action/Pilot message.
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
cola2_msgs__action__Pilot_Result__copy(
  const cola2_msgs__action__Pilot_Result * input,
  cola2_msgs__action__Pilot_Result * output);

/// Initialize array of action/Pilot messages.
/**
 * It allocates the memory for the number of elements and calls
 * cola2_msgs__action__Pilot_Result__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
bool
cola2_msgs__action__Pilot_Result__Sequence__init(cola2_msgs__action__Pilot_Result__Sequence * array, size_t size);

/// Finalize array of action/Pilot messages.
/**
 * It calls
 * cola2_msgs__action__Pilot_Result__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
void
cola2_msgs__action__Pilot_Result__Sequence__fini(cola2_msgs__action__Pilot_Result__Sequence * array);

/// Create array of action/Pilot messages.
/**
 * It allocates the memory for the array and calls
 * cola2_msgs__action__Pilot_Result__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
cola2_msgs__action__Pilot_Result__Sequence *
cola2_msgs__action__Pilot_Result__Sequence__create(size_t size);

/// Destroy array of action/Pilot messages.
/**
 * It calls
 * cola2_msgs__action__Pilot_Result__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
void
cola2_msgs__action__Pilot_Result__Sequence__destroy(cola2_msgs__action__Pilot_Result__Sequence * array);

/// Check for action/Pilot message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
bool
cola2_msgs__action__Pilot_Result__Sequence__are_equal(const cola2_msgs__action__Pilot_Result__Sequence * lhs, const cola2_msgs__action__Pilot_Result__Sequence * rhs);

/// Copy an array of action/Pilot messages.
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
cola2_msgs__action__Pilot_Result__Sequence__copy(
  const cola2_msgs__action__Pilot_Result__Sequence * input,
  cola2_msgs__action__Pilot_Result__Sequence * output);

/// Initialize action/Pilot message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * cola2_msgs__action__Pilot_Feedback
 * )) before or use
 * cola2_msgs__action__Pilot_Feedback__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
bool
cola2_msgs__action__Pilot_Feedback__init(cola2_msgs__action__Pilot_Feedback * msg);

/// Finalize action/Pilot message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
void
cola2_msgs__action__Pilot_Feedback__fini(cola2_msgs__action__Pilot_Feedback * msg);

/// Create action/Pilot message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * cola2_msgs__action__Pilot_Feedback__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
cola2_msgs__action__Pilot_Feedback *
cola2_msgs__action__Pilot_Feedback__create();

/// Destroy action/Pilot message.
/**
 * It calls
 * cola2_msgs__action__Pilot_Feedback__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
void
cola2_msgs__action__Pilot_Feedback__destroy(cola2_msgs__action__Pilot_Feedback * msg);

/// Check for action/Pilot message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
bool
cola2_msgs__action__Pilot_Feedback__are_equal(const cola2_msgs__action__Pilot_Feedback * lhs, const cola2_msgs__action__Pilot_Feedback * rhs);

/// Copy a action/Pilot message.
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
cola2_msgs__action__Pilot_Feedback__copy(
  const cola2_msgs__action__Pilot_Feedback * input,
  cola2_msgs__action__Pilot_Feedback * output);

/// Initialize array of action/Pilot messages.
/**
 * It allocates the memory for the number of elements and calls
 * cola2_msgs__action__Pilot_Feedback__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
bool
cola2_msgs__action__Pilot_Feedback__Sequence__init(cola2_msgs__action__Pilot_Feedback__Sequence * array, size_t size);

/// Finalize array of action/Pilot messages.
/**
 * It calls
 * cola2_msgs__action__Pilot_Feedback__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
void
cola2_msgs__action__Pilot_Feedback__Sequence__fini(cola2_msgs__action__Pilot_Feedback__Sequence * array);

/// Create array of action/Pilot messages.
/**
 * It allocates the memory for the array and calls
 * cola2_msgs__action__Pilot_Feedback__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
cola2_msgs__action__Pilot_Feedback__Sequence *
cola2_msgs__action__Pilot_Feedback__Sequence__create(size_t size);

/// Destroy array of action/Pilot messages.
/**
 * It calls
 * cola2_msgs__action__Pilot_Feedback__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
void
cola2_msgs__action__Pilot_Feedback__Sequence__destroy(cola2_msgs__action__Pilot_Feedback__Sequence * array);

/// Check for action/Pilot message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
bool
cola2_msgs__action__Pilot_Feedback__Sequence__are_equal(const cola2_msgs__action__Pilot_Feedback__Sequence * lhs, const cola2_msgs__action__Pilot_Feedback__Sequence * rhs);

/// Copy an array of action/Pilot messages.
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
cola2_msgs__action__Pilot_Feedback__Sequence__copy(
  const cola2_msgs__action__Pilot_Feedback__Sequence * input,
  cola2_msgs__action__Pilot_Feedback__Sequence * output);

/// Initialize action/Pilot message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * cola2_msgs__action__Pilot_SendGoal_Request
 * )) before or use
 * cola2_msgs__action__Pilot_SendGoal_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
bool
cola2_msgs__action__Pilot_SendGoal_Request__init(cola2_msgs__action__Pilot_SendGoal_Request * msg);

/// Finalize action/Pilot message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
void
cola2_msgs__action__Pilot_SendGoal_Request__fini(cola2_msgs__action__Pilot_SendGoal_Request * msg);

/// Create action/Pilot message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * cola2_msgs__action__Pilot_SendGoal_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
cola2_msgs__action__Pilot_SendGoal_Request *
cola2_msgs__action__Pilot_SendGoal_Request__create();

/// Destroy action/Pilot message.
/**
 * It calls
 * cola2_msgs__action__Pilot_SendGoal_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
void
cola2_msgs__action__Pilot_SendGoal_Request__destroy(cola2_msgs__action__Pilot_SendGoal_Request * msg);

/// Check for action/Pilot message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
bool
cola2_msgs__action__Pilot_SendGoal_Request__are_equal(const cola2_msgs__action__Pilot_SendGoal_Request * lhs, const cola2_msgs__action__Pilot_SendGoal_Request * rhs);

/// Copy a action/Pilot message.
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
cola2_msgs__action__Pilot_SendGoal_Request__copy(
  const cola2_msgs__action__Pilot_SendGoal_Request * input,
  cola2_msgs__action__Pilot_SendGoal_Request * output);

/// Initialize array of action/Pilot messages.
/**
 * It allocates the memory for the number of elements and calls
 * cola2_msgs__action__Pilot_SendGoal_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
bool
cola2_msgs__action__Pilot_SendGoal_Request__Sequence__init(cola2_msgs__action__Pilot_SendGoal_Request__Sequence * array, size_t size);

/// Finalize array of action/Pilot messages.
/**
 * It calls
 * cola2_msgs__action__Pilot_SendGoal_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
void
cola2_msgs__action__Pilot_SendGoal_Request__Sequence__fini(cola2_msgs__action__Pilot_SendGoal_Request__Sequence * array);

/// Create array of action/Pilot messages.
/**
 * It allocates the memory for the array and calls
 * cola2_msgs__action__Pilot_SendGoal_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
cola2_msgs__action__Pilot_SendGoal_Request__Sequence *
cola2_msgs__action__Pilot_SendGoal_Request__Sequence__create(size_t size);

/// Destroy array of action/Pilot messages.
/**
 * It calls
 * cola2_msgs__action__Pilot_SendGoal_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
void
cola2_msgs__action__Pilot_SendGoal_Request__Sequence__destroy(cola2_msgs__action__Pilot_SendGoal_Request__Sequence * array);

/// Check for action/Pilot message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
bool
cola2_msgs__action__Pilot_SendGoal_Request__Sequence__are_equal(const cola2_msgs__action__Pilot_SendGoal_Request__Sequence * lhs, const cola2_msgs__action__Pilot_SendGoal_Request__Sequence * rhs);

/// Copy an array of action/Pilot messages.
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
cola2_msgs__action__Pilot_SendGoal_Request__Sequence__copy(
  const cola2_msgs__action__Pilot_SendGoal_Request__Sequence * input,
  cola2_msgs__action__Pilot_SendGoal_Request__Sequence * output);

/// Initialize action/Pilot message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * cola2_msgs__action__Pilot_SendGoal_Response
 * )) before or use
 * cola2_msgs__action__Pilot_SendGoal_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
bool
cola2_msgs__action__Pilot_SendGoal_Response__init(cola2_msgs__action__Pilot_SendGoal_Response * msg);

/// Finalize action/Pilot message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
void
cola2_msgs__action__Pilot_SendGoal_Response__fini(cola2_msgs__action__Pilot_SendGoal_Response * msg);

/// Create action/Pilot message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * cola2_msgs__action__Pilot_SendGoal_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
cola2_msgs__action__Pilot_SendGoal_Response *
cola2_msgs__action__Pilot_SendGoal_Response__create();

/// Destroy action/Pilot message.
/**
 * It calls
 * cola2_msgs__action__Pilot_SendGoal_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
void
cola2_msgs__action__Pilot_SendGoal_Response__destroy(cola2_msgs__action__Pilot_SendGoal_Response * msg);

/// Check for action/Pilot message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
bool
cola2_msgs__action__Pilot_SendGoal_Response__are_equal(const cola2_msgs__action__Pilot_SendGoal_Response * lhs, const cola2_msgs__action__Pilot_SendGoal_Response * rhs);

/// Copy a action/Pilot message.
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
cola2_msgs__action__Pilot_SendGoal_Response__copy(
  const cola2_msgs__action__Pilot_SendGoal_Response * input,
  cola2_msgs__action__Pilot_SendGoal_Response * output);

/// Initialize array of action/Pilot messages.
/**
 * It allocates the memory for the number of elements and calls
 * cola2_msgs__action__Pilot_SendGoal_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
bool
cola2_msgs__action__Pilot_SendGoal_Response__Sequence__init(cola2_msgs__action__Pilot_SendGoal_Response__Sequence * array, size_t size);

/// Finalize array of action/Pilot messages.
/**
 * It calls
 * cola2_msgs__action__Pilot_SendGoal_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
void
cola2_msgs__action__Pilot_SendGoal_Response__Sequence__fini(cola2_msgs__action__Pilot_SendGoal_Response__Sequence * array);

/// Create array of action/Pilot messages.
/**
 * It allocates the memory for the array and calls
 * cola2_msgs__action__Pilot_SendGoal_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
cola2_msgs__action__Pilot_SendGoal_Response__Sequence *
cola2_msgs__action__Pilot_SendGoal_Response__Sequence__create(size_t size);

/// Destroy array of action/Pilot messages.
/**
 * It calls
 * cola2_msgs__action__Pilot_SendGoal_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
void
cola2_msgs__action__Pilot_SendGoal_Response__Sequence__destroy(cola2_msgs__action__Pilot_SendGoal_Response__Sequence * array);

/// Check for action/Pilot message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
bool
cola2_msgs__action__Pilot_SendGoal_Response__Sequence__are_equal(const cola2_msgs__action__Pilot_SendGoal_Response__Sequence * lhs, const cola2_msgs__action__Pilot_SendGoal_Response__Sequence * rhs);

/// Copy an array of action/Pilot messages.
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
cola2_msgs__action__Pilot_SendGoal_Response__Sequence__copy(
  const cola2_msgs__action__Pilot_SendGoal_Response__Sequence * input,
  cola2_msgs__action__Pilot_SendGoal_Response__Sequence * output);

/// Initialize action/Pilot message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * cola2_msgs__action__Pilot_GetResult_Request
 * )) before or use
 * cola2_msgs__action__Pilot_GetResult_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
bool
cola2_msgs__action__Pilot_GetResult_Request__init(cola2_msgs__action__Pilot_GetResult_Request * msg);

/// Finalize action/Pilot message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
void
cola2_msgs__action__Pilot_GetResult_Request__fini(cola2_msgs__action__Pilot_GetResult_Request * msg);

/// Create action/Pilot message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * cola2_msgs__action__Pilot_GetResult_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
cola2_msgs__action__Pilot_GetResult_Request *
cola2_msgs__action__Pilot_GetResult_Request__create();

/// Destroy action/Pilot message.
/**
 * It calls
 * cola2_msgs__action__Pilot_GetResult_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
void
cola2_msgs__action__Pilot_GetResult_Request__destroy(cola2_msgs__action__Pilot_GetResult_Request * msg);

/// Check for action/Pilot message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
bool
cola2_msgs__action__Pilot_GetResult_Request__are_equal(const cola2_msgs__action__Pilot_GetResult_Request * lhs, const cola2_msgs__action__Pilot_GetResult_Request * rhs);

/// Copy a action/Pilot message.
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
cola2_msgs__action__Pilot_GetResult_Request__copy(
  const cola2_msgs__action__Pilot_GetResult_Request * input,
  cola2_msgs__action__Pilot_GetResult_Request * output);

/// Initialize array of action/Pilot messages.
/**
 * It allocates the memory for the number of elements and calls
 * cola2_msgs__action__Pilot_GetResult_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
bool
cola2_msgs__action__Pilot_GetResult_Request__Sequence__init(cola2_msgs__action__Pilot_GetResult_Request__Sequence * array, size_t size);

/// Finalize array of action/Pilot messages.
/**
 * It calls
 * cola2_msgs__action__Pilot_GetResult_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
void
cola2_msgs__action__Pilot_GetResult_Request__Sequence__fini(cola2_msgs__action__Pilot_GetResult_Request__Sequence * array);

/// Create array of action/Pilot messages.
/**
 * It allocates the memory for the array and calls
 * cola2_msgs__action__Pilot_GetResult_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
cola2_msgs__action__Pilot_GetResult_Request__Sequence *
cola2_msgs__action__Pilot_GetResult_Request__Sequence__create(size_t size);

/// Destroy array of action/Pilot messages.
/**
 * It calls
 * cola2_msgs__action__Pilot_GetResult_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
void
cola2_msgs__action__Pilot_GetResult_Request__Sequence__destroy(cola2_msgs__action__Pilot_GetResult_Request__Sequence * array);

/// Check for action/Pilot message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
bool
cola2_msgs__action__Pilot_GetResult_Request__Sequence__are_equal(const cola2_msgs__action__Pilot_GetResult_Request__Sequence * lhs, const cola2_msgs__action__Pilot_GetResult_Request__Sequence * rhs);

/// Copy an array of action/Pilot messages.
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
cola2_msgs__action__Pilot_GetResult_Request__Sequence__copy(
  const cola2_msgs__action__Pilot_GetResult_Request__Sequence * input,
  cola2_msgs__action__Pilot_GetResult_Request__Sequence * output);

/// Initialize action/Pilot message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * cola2_msgs__action__Pilot_GetResult_Response
 * )) before or use
 * cola2_msgs__action__Pilot_GetResult_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
bool
cola2_msgs__action__Pilot_GetResult_Response__init(cola2_msgs__action__Pilot_GetResult_Response * msg);

/// Finalize action/Pilot message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
void
cola2_msgs__action__Pilot_GetResult_Response__fini(cola2_msgs__action__Pilot_GetResult_Response * msg);

/// Create action/Pilot message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * cola2_msgs__action__Pilot_GetResult_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
cola2_msgs__action__Pilot_GetResult_Response *
cola2_msgs__action__Pilot_GetResult_Response__create();

/// Destroy action/Pilot message.
/**
 * It calls
 * cola2_msgs__action__Pilot_GetResult_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
void
cola2_msgs__action__Pilot_GetResult_Response__destroy(cola2_msgs__action__Pilot_GetResult_Response * msg);

/// Check for action/Pilot message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
bool
cola2_msgs__action__Pilot_GetResult_Response__are_equal(const cola2_msgs__action__Pilot_GetResult_Response * lhs, const cola2_msgs__action__Pilot_GetResult_Response * rhs);

/// Copy a action/Pilot message.
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
cola2_msgs__action__Pilot_GetResult_Response__copy(
  const cola2_msgs__action__Pilot_GetResult_Response * input,
  cola2_msgs__action__Pilot_GetResult_Response * output);

/// Initialize array of action/Pilot messages.
/**
 * It allocates the memory for the number of elements and calls
 * cola2_msgs__action__Pilot_GetResult_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
bool
cola2_msgs__action__Pilot_GetResult_Response__Sequence__init(cola2_msgs__action__Pilot_GetResult_Response__Sequence * array, size_t size);

/// Finalize array of action/Pilot messages.
/**
 * It calls
 * cola2_msgs__action__Pilot_GetResult_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
void
cola2_msgs__action__Pilot_GetResult_Response__Sequence__fini(cola2_msgs__action__Pilot_GetResult_Response__Sequence * array);

/// Create array of action/Pilot messages.
/**
 * It allocates the memory for the array and calls
 * cola2_msgs__action__Pilot_GetResult_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
cola2_msgs__action__Pilot_GetResult_Response__Sequence *
cola2_msgs__action__Pilot_GetResult_Response__Sequence__create(size_t size);

/// Destroy array of action/Pilot messages.
/**
 * It calls
 * cola2_msgs__action__Pilot_GetResult_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
void
cola2_msgs__action__Pilot_GetResult_Response__Sequence__destroy(cola2_msgs__action__Pilot_GetResult_Response__Sequence * array);

/// Check for action/Pilot message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
bool
cola2_msgs__action__Pilot_GetResult_Response__Sequence__are_equal(const cola2_msgs__action__Pilot_GetResult_Response__Sequence * lhs, const cola2_msgs__action__Pilot_GetResult_Response__Sequence * rhs);

/// Copy an array of action/Pilot messages.
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
cola2_msgs__action__Pilot_GetResult_Response__Sequence__copy(
  const cola2_msgs__action__Pilot_GetResult_Response__Sequence * input,
  cola2_msgs__action__Pilot_GetResult_Response__Sequence * output);

/// Initialize action/Pilot message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * cola2_msgs__action__Pilot_FeedbackMessage
 * )) before or use
 * cola2_msgs__action__Pilot_FeedbackMessage__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
bool
cola2_msgs__action__Pilot_FeedbackMessage__init(cola2_msgs__action__Pilot_FeedbackMessage * msg);

/// Finalize action/Pilot message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
void
cola2_msgs__action__Pilot_FeedbackMessage__fini(cola2_msgs__action__Pilot_FeedbackMessage * msg);

/// Create action/Pilot message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * cola2_msgs__action__Pilot_FeedbackMessage__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
cola2_msgs__action__Pilot_FeedbackMessage *
cola2_msgs__action__Pilot_FeedbackMessage__create();

/// Destroy action/Pilot message.
/**
 * It calls
 * cola2_msgs__action__Pilot_FeedbackMessage__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
void
cola2_msgs__action__Pilot_FeedbackMessage__destroy(cola2_msgs__action__Pilot_FeedbackMessage * msg);

/// Check for action/Pilot message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
bool
cola2_msgs__action__Pilot_FeedbackMessage__are_equal(const cola2_msgs__action__Pilot_FeedbackMessage * lhs, const cola2_msgs__action__Pilot_FeedbackMessage * rhs);

/// Copy a action/Pilot message.
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
cola2_msgs__action__Pilot_FeedbackMessage__copy(
  const cola2_msgs__action__Pilot_FeedbackMessage * input,
  cola2_msgs__action__Pilot_FeedbackMessage * output);

/// Initialize array of action/Pilot messages.
/**
 * It allocates the memory for the number of elements and calls
 * cola2_msgs__action__Pilot_FeedbackMessage__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
bool
cola2_msgs__action__Pilot_FeedbackMessage__Sequence__init(cola2_msgs__action__Pilot_FeedbackMessage__Sequence * array, size_t size);

/// Finalize array of action/Pilot messages.
/**
 * It calls
 * cola2_msgs__action__Pilot_FeedbackMessage__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
void
cola2_msgs__action__Pilot_FeedbackMessage__Sequence__fini(cola2_msgs__action__Pilot_FeedbackMessage__Sequence * array);

/// Create array of action/Pilot messages.
/**
 * It allocates the memory for the array and calls
 * cola2_msgs__action__Pilot_FeedbackMessage__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
cola2_msgs__action__Pilot_FeedbackMessage__Sequence *
cola2_msgs__action__Pilot_FeedbackMessage__Sequence__create(size_t size);

/// Destroy array of action/Pilot messages.
/**
 * It calls
 * cola2_msgs__action__Pilot_FeedbackMessage__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
void
cola2_msgs__action__Pilot_FeedbackMessage__Sequence__destroy(cola2_msgs__action__Pilot_FeedbackMessage__Sequence * array);

/// Check for action/Pilot message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_cola2_msgs
bool
cola2_msgs__action__Pilot_FeedbackMessage__Sequence__are_equal(const cola2_msgs__action__Pilot_FeedbackMessage__Sequence * lhs, const cola2_msgs__action__Pilot_FeedbackMessage__Sequence * rhs);

/// Copy an array of action/Pilot messages.
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
cola2_msgs__action__Pilot_FeedbackMessage__Sequence__copy(
  const cola2_msgs__action__Pilot_FeedbackMessage__Sequence * input,
  cola2_msgs__action__Pilot_FeedbackMessage__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // COLA2_MSGS__ACTION__DETAIL__PILOT__FUNCTIONS_H_
