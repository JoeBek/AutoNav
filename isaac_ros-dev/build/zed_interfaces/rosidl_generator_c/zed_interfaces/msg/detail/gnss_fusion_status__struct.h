// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from zed_interfaces:msg/GnssFusionStatus.idl
// generated code does not contain a copyright notice

#ifndef ZED_INTERFACES__MSG__DETAIL__GNSS_FUSION_STATUS__STRUCT_H_
#define ZED_INTERFACES__MSG__DETAIL__GNSS_FUSION_STATUS__STRUCT_H_

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
  zed_interfaces__msg__GnssFusionStatus__OK = 0
};

/// Constant 'OFF'.
enum
{
  zed_interfaces__msg__GnssFusionStatus__OFF = 1
};

/// Constant 'CALIBRATION_IN_PROGRESS'.
enum
{
  zed_interfaces__msg__GnssFusionStatus__CALIBRATION_IN_PROGRESS = 2
};

/// Constant 'RECALIBRATION_IN_PROGRESS'.
enum
{
  zed_interfaces__msg__GnssFusionStatus__RECALIBRATION_IN_PROGRESS = 3
};

/// Struct defined in msg/GnssFusionStatus in the package zed_interfaces.
/**
  * GNSS_FUSION_STATUS
  * Represents the current state of GNSS fusion for global localization.
  * OK - The GNSS fusion module is calibrated and working successfully.
  * OFF - The GNSS fusion module is not enabled.
  * CALIBRATION_IN_PROGRESS - Calibration of the GNSS/VIO fusion module is in progress.
  * RECALIBRATION_IN_PROGRESS- Re-alignment of GNSS/VIO data is in progress, leading to potentially inaccurate global position.
 */
typedef struct zed_interfaces__msg__GnssFusionStatus
{
  uint8_t gnss_fusion_status;
} zed_interfaces__msg__GnssFusionStatus;

// Struct for a sequence of zed_interfaces__msg__GnssFusionStatus.
typedef struct zed_interfaces__msg__GnssFusionStatus__Sequence
{
  zed_interfaces__msg__GnssFusionStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} zed_interfaces__msg__GnssFusionStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ZED_INTERFACES__MSG__DETAIL__GNSS_FUSION_STATUS__STRUCT_H_
