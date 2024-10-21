// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonav_interfaces:msg/Object.idl
// generated code does not contain a copyright notice

#ifndef AUTONAV_INTERFACES__MSG__DETAIL__OBJECT__STRUCT_H_
#define AUTONAV_INTERFACES__MSG__DETAIL__OBJECT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Object in the package autonav_interfaces.
typedef struct autonav_interfaces__msg__Object
{
  double x;
  double z;
} autonav_interfaces__msg__Object;

// Struct for a sequence of autonav_interfaces__msg__Object.
typedef struct autonav_interfaces__msg__Object__Sequence
{
  autonav_interfaces__msg__Object * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonav_interfaces__msg__Object__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONAV_INTERFACES__MSG__DETAIL__OBJECT__STRUCT_H_
