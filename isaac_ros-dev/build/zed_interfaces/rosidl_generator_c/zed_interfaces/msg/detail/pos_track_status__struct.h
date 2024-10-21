// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from zed_interfaces:msg/PosTrackStatus.idl
// generated code does not contain a copyright notice

#ifndef ZED_INTERFACES__MSG__DETAIL__POS_TRACK_STATUS__STRUCT_H_
#define ZED_INTERFACES__MSG__DETAIL__POS_TRACK_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'OK'.
enum
{
  zed_interfaces__msg__PosTrackStatus__OK = 0
};

/// Constant 'UNAVAILABLE'.
enum
{
  zed_interfaces__msg__PosTrackStatus__UNAVAILABLE = 1
};

/// Constant 'LOOP_CLOSED'.
enum
{
  zed_interfaces__msg__PosTrackStatus__LOOP_CLOSED = 1
};

/// Constant 'SEARCHING'.
enum
{
  zed_interfaces__msg__PosTrackStatus__SEARCHING = 2
};

/// Constant 'OFF'.
enum
{
  zed_interfaces__msg__PosTrackStatus__OFF = 3
};

/// Struct defined in msg/PosTrackStatus in the package zed_interfaces.
/**
  * CONSTANTS
 */
typedef struct zed_interfaces__msg__PosTrackStatus
{
  /// VISUAL_STATUS
  /// Represents the current state of Visual-Inertial Odometry (VIO) tracking between the previous frame and the current frame.
  /// OK  - The positional tracking module successfully tracked from the previous frame to the current frame.
  /// NOT_OK - The positional tracking module failed to track from the previous frame to the current frame.
  uint8_t odometry_status;
  /// SPATIAL_MEMORY_STATUS
  /// Represents the current state of camera tracking in the global map.
  /// OK - The positional tracking module is operating normally.
  /// LOOP_CLOSED - The positional tracking module detected a loop and corrected its position.
  /// SEARCHING - The positional tracking module is searching for recognizable areas in the global map to relocate.
  /// OFF - Spatial memory is disabled
  uint8_t spatial_memory_status;
  /// Deprecated
  uint8_t status;
} zed_interfaces__msg__PosTrackStatus;

// Struct for a sequence of zed_interfaces__msg__PosTrackStatus.
typedef struct zed_interfaces__msg__PosTrackStatus__Sequence
{
  zed_interfaces__msg__PosTrackStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} zed_interfaces__msg__PosTrackStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ZED_INTERFACES__MSG__DETAIL__POS_TRACK_STATUS__STRUCT_H_
