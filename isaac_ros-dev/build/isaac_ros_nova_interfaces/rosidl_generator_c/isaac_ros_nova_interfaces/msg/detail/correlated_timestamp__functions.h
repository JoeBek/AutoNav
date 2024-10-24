// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from isaac_ros_nova_interfaces:msg/CorrelatedTimestamp.idl
// generated code does not contain a copyright notice

#ifndef ISAAC_ROS_NOVA_INTERFACES__MSG__DETAIL__CORRELATED_TIMESTAMP__FUNCTIONS_H_
#define ISAAC_ROS_NOVA_INTERFACES__MSG__DETAIL__CORRELATED_TIMESTAMP__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "isaac_ros_nova_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "isaac_ros_nova_interfaces/msg/detail/correlated_timestamp__struct.h"

/// Initialize msg/CorrelatedTimestamp message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * isaac_ros_nova_interfaces__msg__CorrelatedTimestamp
 * )) before or use
 * isaac_ros_nova_interfaces__msg__CorrelatedTimestamp__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_isaac_ros_nova_interfaces
bool
isaac_ros_nova_interfaces__msg__CorrelatedTimestamp__init(isaac_ros_nova_interfaces__msg__CorrelatedTimestamp * msg);

/// Finalize msg/CorrelatedTimestamp message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_isaac_ros_nova_interfaces
void
isaac_ros_nova_interfaces__msg__CorrelatedTimestamp__fini(isaac_ros_nova_interfaces__msg__CorrelatedTimestamp * msg);

/// Create msg/CorrelatedTimestamp message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * isaac_ros_nova_interfaces__msg__CorrelatedTimestamp__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_isaac_ros_nova_interfaces
isaac_ros_nova_interfaces__msg__CorrelatedTimestamp *
isaac_ros_nova_interfaces__msg__CorrelatedTimestamp__create();

/// Destroy msg/CorrelatedTimestamp message.
/**
 * It calls
 * isaac_ros_nova_interfaces__msg__CorrelatedTimestamp__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_isaac_ros_nova_interfaces
void
isaac_ros_nova_interfaces__msg__CorrelatedTimestamp__destroy(isaac_ros_nova_interfaces__msg__CorrelatedTimestamp * msg);

/// Check for msg/CorrelatedTimestamp message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_isaac_ros_nova_interfaces
bool
isaac_ros_nova_interfaces__msg__CorrelatedTimestamp__are_equal(const isaac_ros_nova_interfaces__msg__CorrelatedTimestamp * lhs, const isaac_ros_nova_interfaces__msg__CorrelatedTimestamp * rhs);

/// Copy a msg/CorrelatedTimestamp message.
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
ROSIDL_GENERATOR_C_PUBLIC_isaac_ros_nova_interfaces
bool
isaac_ros_nova_interfaces__msg__CorrelatedTimestamp__copy(
  const isaac_ros_nova_interfaces__msg__CorrelatedTimestamp * input,
  isaac_ros_nova_interfaces__msg__CorrelatedTimestamp * output);

/// Initialize array of msg/CorrelatedTimestamp messages.
/**
 * It allocates the memory for the number of elements and calls
 * isaac_ros_nova_interfaces__msg__CorrelatedTimestamp__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_isaac_ros_nova_interfaces
bool
isaac_ros_nova_interfaces__msg__CorrelatedTimestamp__Sequence__init(isaac_ros_nova_interfaces__msg__CorrelatedTimestamp__Sequence * array, size_t size);

/// Finalize array of msg/CorrelatedTimestamp messages.
/**
 * It calls
 * isaac_ros_nova_interfaces__msg__CorrelatedTimestamp__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_isaac_ros_nova_interfaces
void
isaac_ros_nova_interfaces__msg__CorrelatedTimestamp__Sequence__fini(isaac_ros_nova_interfaces__msg__CorrelatedTimestamp__Sequence * array);

/// Create array of msg/CorrelatedTimestamp messages.
/**
 * It allocates the memory for the array and calls
 * isaac_ros_nova_interfaces__msg__CorrelatedTimestamp__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_isaac_ros_nova_interfaces
isaac_ros_nova_interfaces__msg__CorrelatedTimestamp__Sequence *
isaac_ros_nova_interfaces__msg__CorrelatedTimestamp__Sequence__create(size_t size);

/// Destroy array of msg/CorrelatedTimestamp messages.
/**
 * It calls
 * isaac_ros_nova_interfaces__msg__CorrelatedTimestamp__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_isaac_ros_nova_interfaces
void
isaac_ros_nova_interfaces__msg__CorrelatedTimestamp__Sequence__destroy(isaac_ros_nova_interfaces__msg__CorrelatedTimestamp__Sequence * array);

/// Check for msg/CorrelatedTimestamp message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_isaac_ros_nova_interfaces
bool
isaac_ros_nova_interfaces__msg__CorrelatedTimestamp__Sequence__are_equal(const isaac_ros_nova_interfaces__msg__CorrelatedTimestamp__Sequence * lhs, const isaac_ros_nova_interfaces__msg__CorrelatedTimestamp__Sequence * rhs);

/// Copy an array of msg/CorrelatedTimestamp messages.
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
ROSIDL_GENERATOR_C_PUBLIC_isaac_ros_nova_interfaces
bool
isaac_ros_nova_interfaces__msg__CorrelatedTimestamp__Sequence__copy(
  const isaac_ros_nova_interfaces__msg__CorrelatedTimestamp__Sequence * input,
  isaac_ros_nova_interfaces__msg__CorrelatedTimestamp__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ISAAC_ROS_NOVA_INTERFACES__MSG__DETAIL__CORRELATED_TIMESTAMP__FUNCTIONS_H_
