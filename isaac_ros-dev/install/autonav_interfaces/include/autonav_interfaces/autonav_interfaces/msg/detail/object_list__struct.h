// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonav_interfaces:msg/ObjectList.idl
// generated code does not contain a copyright notice

#ifndef AUTONAV_INTERFACES__MSG__DETAIL__OBJECT_LIST__STRUCT_H_
#define AUTONAV_INTERFACES__MSG__DETAIL__OBJECT_LIST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'objects'
#include "autonav_interfaces/msg/detail/object__struct.h"

/// Struct defined in msg/ObjectList in the package autonav_interfaces.
/**
  * List of objects with X and Z coordinates
 */
typedef struct autonav_interfaces__msg__ObjectList
{
  autonav_interfaces__msg__Object__Sequence objects;
} autonav_interfaces__msg__ObjectList;

// Struct for a sequence of autonav_interfaces__msg__ObjectList.
typedef struct autonav_interfaces__msg__ObjectList__Sequence
{
  autonav_interfaces__msg__ObjectList * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonav_interfaces__msg__ObjectList__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONAV_INTERFACES__MSG__DETAIL__OBJECT_LIST__STRUCT_H_
