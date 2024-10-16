// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from zed_interfaces:msg/MagHeadingStatus.idl
// generated code does not contain a copyright notice

#ifndef ZED_INTERFACES__MSG__DETAIL__MAG_HEADING_STATUS__STRUCT_H_
#define ZED_INTERFACES__MSG__DETAIL__MAG_HEADING_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'GOOD'.
enum
{
  zed_interfaces__msg__MagHeadingStatus__GOOD = 0
};

/// Constant 'OK'.
enum
{
  zed_interfaces__msg__MagHeadingStatus__OK = 1
};

/// Constant 'NOT_GOOD'.
enum
{
  zed_interfaces__msg__MagHeadingStatus__NOT_GOOD = 2
};

/// Constant 'NOT_CALIBRATED'.
enum
{
  zed_interfaces__msg__MagHeadingStatus__NOT_CALIBRATED = 3
};

/// Constant 'MAG_NOT_AVAILABLE'.
enum
{
  zed_interfaces__msg__MagHeadingStatus__MAG_NOT_AVAILABLE = 4
};

/// Struct defined in msg/MagHeadingStatus in the package zed_interfaces.
/**
  * Status constants
  *  GOOD               The heading is reliable and not affected by iron interferences.
  *  OK                 The heading is reliable, but affected by slight iron interferences.
  *  NOT_GOOD           The heading is not reliable because affected by strong iron interferences.
  *  NOT_CALIBRATED     The magnetometer has not been calibrated.
  *  MAG_NOT_AVAILABLE  The magnetometer sensor is not available.
 */
typedef struct zed_interfaces__msg__MagHeadingStatus
{
  /// Status
  uint8_t status;
} zed_interfaces__msg__MagHeadingStatus;

// Struct for a sequence of zed_interfaces__msg__MagHeadingStatus.
typedef struct zed_interfaces__msg__MagHeadingStatus__Sequence
{
  zed_interfaces__msg__MagHeadingStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} zed_interfaces__msg__MagHeadingStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ZED_INTERFACES__MSG__DETAIL__MAG_HEADING_STATUS__STRUCT_H_
