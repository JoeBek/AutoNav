// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from autonav_interfaces:msg/Object.idl
// generated code does not contain a copyright notice

#ifndef AUTONAV_INTERFACES__MSG__DETAIL__OBJECT__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define AUTONAV_INTERFACES__MSG__DETAIL__OBJECT__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "autonav_interfaces/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "autonav_interfaces/msg/detail/object__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace autonav_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_autonav_interfaces
cdr_serialize(
  const autonav_interfaces::msg::Object & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_autonav_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  autonav_interfaces::msg::Object & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_autonav_interfaces
get_serialized_size(
  const autonav_interfaces::msg::Object & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_autonav_interfaces
max_serialized_size_Object(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace autonav_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_autonav_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, autonav_interfaces, msg, Object)();

#ifdef __cplusplus
}
#endif

#endif  // AUTONAV_INTERFACES__MSG__DETAIL__OBJECT__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
