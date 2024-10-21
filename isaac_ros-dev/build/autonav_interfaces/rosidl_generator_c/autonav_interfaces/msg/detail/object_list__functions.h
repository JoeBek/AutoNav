// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from autonav_interfaces:msg/ObjectList.idl
// generated code does not contain a copyright notice

#ifndef AUTONAV_INTERFACES__MSG__DETAIL__OBJECT_LIST__FUNCTIONS_H_
#define AUTONAV_INTERFACES__MSG__DETAIL__OBJECT_LIST__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "autonav_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "autonav_interfaces/msg/detail/object_list__struct.h"

/// Initialize msg/ObjectList message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * autonav_interfaces__msg__ObjectList
 * )) before or use
 * autonav_interfaces__msg__ObjectList__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_autonav_interfaces
bool
autonav_interfaces__msg__ObjectList__init(autonav_interfaces__msg__ObjectList * msg);

/// Finalize msg/ObjectList message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_autonav_interfaces
void
autonav_interfaces__msg__ObjectList__fini(autonav_interfaces__msg__ObjectList * msg);

/// Create msg/ObjectList message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * autonav_interfaces__msg__ObjectList__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_autonav_interfaces
autonav_interfaces__msg__ObjectList *
autonav_interfaces__msg__ObjectList__create();

/// Destroy msg/ObjectList message.
/**
 * It calls
 * autonav_interfaces__msg__ObjectList__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_autonav_interfaces
void
autonav_interfaces__msg__ObjectList__destroy(autonav_interfaces__msg__ObjectList * msg);

/// Check for msg/ObjectList message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_autonav_interfaces
bool
autonav_interfaces__msg__ObjectList__are_equal(const autonav_interfaces__msg__ObjectList * lhs, const autonav_interfaces__msg__ObjectList * rhs);

/// Copy a msg/ObjectList message.
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
ROSIDL_GENERATOR_C_PUBLIC_autonav_interfaces
bool
autonav_interfaces__msg__ObjectList__copy(
  const autonav_interfaces__msg__ObjectList * input,
  autonav_interfaces__msg__ObjectList * output);

/// Initialize array of msg/ObjectList messages.
/**
 * It allocates the memory for the number of elements and calls
 * autonav_interfaces__msg__ObjectList__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_autonav_interfaces
bool
autonav_interfaces__msg__ObjectList__Sequence__init(autonav_interfaces__msg__ObjectList__Sequence * array, size_t size);

/// Finalize array of msg/ObjectList messages.
/**
 * It calls
 * autonav_interfaces__msg__ObjectList__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_autonav_interfaces
void
autonav_interfaces__msg__ObjectList__Sequence__fini(autonav_interfaces__msg__ObjectList__Sequence * array);

/// Create array of msg/ObjectList messages.
/**
 * It allocates the memory for the array and calls
 * autonav_interfaces__msg__ObjectList__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_autonav_interfaces
autonav_interfaces__msg__ObjectList__Sequence *
autonav_interfaces__msg__ObjectList__Sequence__create(size_t size);

/// Destroy array of msg/ObjectList messages.
/**
 * It calls
 * autonav_interfaces__msg__ObjectList__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_autonav_interfaces
void
autonav_interfaces__msg__ObjectList__Sequence__destroy(autonav_interfaces__msg__ObjectList__Sequence * array);

/// Check for msg/ObjectList message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_autonav_interfaces
bool
autonav_interfaces__msg__ObjectList__Sequence__are_equal(const autonav_interfaces__msg__ObjectList__Sequence * lhs, const autonav_interfaces__msg__ObjectList__Sequence * rhs);

/// Copy an array of msg/ObjectList messages.
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
ROSIDL_GENERATOR_C_PUBLIC_autonav_interfaces
bool
autonav_interfaces__msg__ObjectList__Sequence__copy(
  const autonav_interfaces__msg__ObjectList__Sequence * input,
  autonav_interfaces__msg__ObjectList__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // AUTONAV_INTERFACES__MSG__DETAIL__OBJECT_LIST__FUNCTIONS_H_
