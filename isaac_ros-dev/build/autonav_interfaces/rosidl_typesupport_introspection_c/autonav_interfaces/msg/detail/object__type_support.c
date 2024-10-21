// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from autonav_interfaces:msg/Object.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "autonav_interfaces/msg/detail/object__rosidl_typesupport_introspection_c.h"
#include "autonav_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "autonav_interfaces/msg/detail/object__functions.h"
#include "autonav_interfaces/msg/detail/object__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void autonav_interfaces__msg__Object__rosidl_typesupport_introspection_c__Object_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonav_interfaces__msg__Object__init(message_memory);
}

void autonav_interfaces__msg__Object__rosidl_typesupport_introspection_c__Object_fini_function(void * message_memory)
{
  autonav_interfaces__msg__Object__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember autonav_interfaces__msg__Object__rosidl_typesupport_introspection_c__Object_message_member_array[2] = {
  {
    "x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonav_interfaces__msg__Object, x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "z",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonav_interfaces__msg__Object, z),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonav_interfaces__msg__Object__rosidl_typesupport_introspection_c__Object_message_members = {
  "autonav_interfaces__msg",  // message namespace
  "Object",  // message name
  2,  // number of fields
  sizeof(autonav_interfaces__msg__Object),
  autonav_interfaces__msg__Object__rosidl_typesupport_introspection_c__Object_message_member_array,  // message members
  autonav_interfaces__msg__Object__rosidl_typesupport_introspection_c__Object_init_function,  // function to initialize message memory (memory has to be allocated)
  autonav_interfaces__msg__Object__rosidl_typesupport_introspection_c__Object_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonav_interfaces__msg__Object__rosidl_typesupport_introspection_c__Object_message_type_support_handle = {
  0,
  &autonav_interfaces__msg__Object__rosidl_typesupport_introspection_c__Object_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonav_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonav_interfaces, msg, Object)() {
  if (!autonav_interfaces__msg__Object__rosidl_typesupport_introspection_c__Object_message_type_support_handle.typesupport_identifier) {
    autonav_interfaces__msg__Object__rosidl_typesupport_introspection_c__Object_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonav_interfaces__msg__Object__rosidl_typesupport_introspection_c__Object_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
