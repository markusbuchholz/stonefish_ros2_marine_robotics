// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from stonefish_ros2:srv/SonarSettings2.idl
// generated code does not contain a copyright notice

#ifndef STONEFISH_ROS2__SRV__DETAIL__SONAR_SETTINGS2__FUNCTIONS_H_
#define STONEFISH_ROS2__SRV__DETAIL__SONAR_SETTINGS2__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "stonefish_ros2/msg/rosidl_generator_c__visibility_control.h"

#include "stonefish_ros2/srv/detail/sonar_settings2__struct.h"

/// Initialize srv/SonarSettings2 message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * stonefish_ros2__srv__SonarSettings2_Request
 * )) before or use
 * stonefish_ros2__srv__SonarSettings2_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_stonefish_ros2
bool
stonefish_ros2__srv__SonarSettings2_Request__init(stonefish_ros2__srv__SonarSettings2_Request * msg);

/// Finalize srv/SonarSettings2 message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stonefish_ros2
void
stonefish_ros2__srv__SonarSettings2_Request__fini(stonefish_ros2__srv__SonarSettings2_Request * msg);

/// Create srv/SonarSettings2 message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * stonefish_ros2__srv__SonarSettings2_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_stonefish_ros2
stonefish_ros2__srv__SonarSettings2_Request *
stonefish_ros2__srv__SonarSettings2_Request__create();

/// Destroy srv/SonarSettings2 message.
/**
 * It calls
 * stonefish_ros2__srv__SonarSettings2_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stonefish_ros2
void
stonefish_ros2__srv__SonarSettings2_Request__destroy(stonefish_ros2__srv__SonarSettings2_Request * msg);

/// Check for srv/SonarSettings2 message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_stonefish_ros2
bool
stonefish_ros2__srv__SonarSettings2_Request__are_equal(const stonefish_ros2__srv__SonarSettings2_Request * lhs, const stonefish_ros2__srv__SonarSettings2_Request * rhs);

/// Copy a srv/SonarSettings2 message.
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
stonefish_ros2__srv__SonarSettings2_Request__copy(
  const stonefish_ros2__srv__SonarSettings2_Request * input,
  stonefish_ros2__srv__SonarSettings2_Request * output);

/// Initialize array of srv/SonarSettings2 messages.
/**
 * It allocates the memory for the number of elements and calls
 * stonefish_ros2__srv__SonarSettings2_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_stonefish_ros2
bool
stonefish_ros2__srv__SonarSettings2_Request__Sequence__init(stonefish_ros2__srv__SonarSettings2_Request__Sequence * array, size_t size);

/// Finalize array of srv/SonarSettings2 messages.
/**
 * It calls
 * stonefish_ros2__srv__SonarSettings2_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stonefish_ros2
void
stonefish_ros2__srv__SonarSettings2_Request__Sequence__fini(stonefish_ros2__srv__SonarSettings2_Request__Sequence * array);

/// Create array of srv/SonarSettings2 messages.
/**
 * It allocates the memory for the array and calls
 * stonefish_ros2__srv__SonarSettings2_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_stonefish_ros2
stonefish_ros2__srv__SonarSettings2_Request__Sequence *
stonefish_ros2__srv__SonarSettings2_Request__Sequence__create(size_t size);

/// Destroy array of srv/SonarSettings2 messages.
/**
 * It calls
 * stonefish_ros2__srv__SonarSettings2_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stonefish_ros2
void
stonefish_ros2__srv__SonarSettings2_Request__Sequence__destroy(stonefish_ros2__srv__SonarSettings2_Request__Sequence * array);

/// Check for srv/SonarSettings2 message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_stonefish_ros2
bool
stonefish_ros2__srv__SonarSettings2_Request__Sequence__are_equal(const stonefish_ros2__srv__SonarSettings2_Request__Sequence * lhs, const stonefish_ros2__srv__SonarSettings2_Request__Sequence * rhs);

/// Copy an array of srv/SonarSettings2 messages.
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
stonefish_ros2__srv__SonarSettings2_Request__Sequence__copy(
  const stonefish_ros2__srv__SonarSettings2_Request__Sequence * input,
  stonefish_ros2__srv__SonarSettings2_Request__Sequence * output);

/// Initialize srv/SonarSettings2 message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * stonefish_ros2__srv__SonarSettings2_Response
 * )) before or use
 * stonefish_ros2__srv__SonarSettings2_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_stonefish_ros2
bool
stonefish_ros2__srv__SonarSettings2_Response__init(stonefish_ros2__srv__SonarSettings2_Response * msg);

/// Finalize srv/SonarSettings2 message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stonefish_ros2
void
stonefish_ros2__srv__SonarSettings2_Response__fini(stonefish_ros2__srv__SonarSettings2_Response * msg);

/// Create srv/SonarSettings2 message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * stonefish_ros2__srv__SonarSettings2_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_stonefish_ros2
stonefish_ros2__srv__SonarSettings2_Response *
stonefish_ros2__srv__SonarSettings2_Response__create();

/// Destroy srv/SonarSettings2 message.
/**
 * It calls
 * stonefish_ros2__srv__SonarSettings2_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stonefish_ros2
void
stonefish_ros2__srv__SonarSettings2_Response__destroy(stonefish_ros2__srv__SonarSettings2_Response * msg);

/// Check for srv/SonarSettings2 message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_stonefish_ros2
bool
stonefish_ros2__srv__SonarSettings2_Response__are_equal(const stonefish_ros2__srv__SonarSettings2_Response * lhs, const stonefish_ros2__srv__SonarSettings2_Response * rhs);

/// Copy a srv/SonarSettings2 message.
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
stonefish_ros2__srv__SonarSettings2_Response__copy(
  const stonefish_ros2__srv__SonarSettings2_Response * input,
  stonefish_ros2__srv__SonarSettings2_Response * output);

/// Initialize array of srv/SonarSettings2 messages.
/**
 * It allocates the memory for the number of elements and calls
 * stonefish_ros2__srv__SonarSettings2_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_stonefish_ros2
bool
stonefish_ros2__srv__SonarSettings2_Response__Sequence__init(stonefish_ros2__srv__SonarSettings2_Response__Sequence * array, size_t size);

/// Finalize array of srv/SonarSettings2 messages.
/**
 * It calls
 * stonefish_ros2__srv__SonarSettings2_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stonefish_ros2
void
stonefish_ros2__srv__SonarSettings2_Response__Sequence__fini(stonefish_ros2__srv__SonarSettings2_Response__Sequence * array);

/// Create array of srv/SonarSettings2 messages.
/**
 * It allocates the memory for the array and calls
 * stonefish_ros2__srv__SonarSettings2_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_stonefish_ros2
stonefish_ros2__srv__SonarSettings2_Response__Sequence *
stonefish_ros2__srv__SonarSettings2_Response__Sequence__create(size_t size);

/// Destroy array of srv/SonarSettings2 messages.
/**
 * It calls
 * stonefish_ros2__srv__SonarSettings2_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stonefish_ros2
void
stonefish_ros2__srv__SonarSettings2_Response__Sequence__destroy(stonefish_ros2__srv__SonarSettings2_Response__Sequence * array);

/// Check for srv/SonarSettings2 message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_stonefish_ros2
bool
stonefish_ros2__srv__SonarSettings2_Response__Sequence__are_equal(const stonefish_ros2__srv__SonarSettings2_Response__Sequence * lhs, const stonefish_ros2__srv__SonarSettings2_Response__Sequence * rhs);

/// Copy an array of srv/SonarSettings2 messages.
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
stonefish_ros2__srv__SonarSettings2_Response__Sequence__copy(
  const stonefish_ros2__srv__SonarSettings2_Response__Sequence * input,
  stonefish_ros2__srv__SonarSettings2_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // STONEFISH_ROS2__SRV__DETAIL__SONAR_SETTINGS2__FUNCTIONS_H_