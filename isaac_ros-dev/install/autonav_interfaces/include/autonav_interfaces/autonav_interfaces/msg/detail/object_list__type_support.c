// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from autonav_interfaces:msg/ObjectList.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "autonav_interfaces/msg/detail/object_list__rosidl_typesupport_introspection_c.h"
#include "autonav_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "autonav_interfaces/msg/detail/object_list__functions.h"
#include "autonav_interfaces/msg/detail/object_list__struct.h"


// Include directives for member types
// Member `objects`
#include "autonav_interfaces/msg/object.h"
// Member `objects`
#include "autonav_interfaces/msg/detail/object__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autonav_interfaces__msg__ObjectList__rosidl_typesupport_introspection_c__ObjectList_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autonav_interfaces__msg__ObjectList__init(message_memory);
}

void autonav_interfaces__msg__ObjectList__rosidl_typesupport_introspection_c__ObjectList_fini_function(void * message_memory)
{
  autonav_interfaces__msg__ObjectList__fini(message_memory);
}

size_t autonav_interfaces__msg__ObjectList__rosidl_typesupport_introspection_c__size_function__ObjectList__objects(
  const void * untyped_member)
{
  const autonav_interfaces__msg__Object__Sequence * member =
    (const autonav_interfaces__msg__Object__Sequence *)(untyped_member);
  return member->size;
}

const void * autonav_interfaces__msg__ObjectList__rosidl_typesupport_introspection_c__get_const_function__ObjectList__objects(
  const void * untyped_member, size_t index)
{
  const autonav_interfaces__msg__Object__Sequence * member =
    (const autonav_interfaces__msg__Object__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autonav_interfaces__msg__ObjectList__rosidl_typesupport_introspection_c__get_function__ObjectList__objects(
  void * untyped_member, size_t index)
{
  autonav_interfaces__msg__Object__Sequence * member =
    (autonav_interfaces__msg__Object__Sequence *)(untyped_member);
  return &member->data[index];
}

void autonav_interfaces__msg__ObjectList__rosidl_typesupport_introspection_c__fetch_function__ObjectList__objects(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const autonav_interfaces__msg__Object * item =
    ((const autonav_interfaces__msg__Object *)
    autonav_interfaces__msg__ObjectList__rosidl_typesupport_introspection_c__get_const_function__ObjectList__objects(untyped_member, index));
  autonav_interfaces__msg__Object * value =
    (autonav_interfaces__msg__Object *)(untyped_value);
  *value = *item;
}

void autonav_interfaces__msg__ObjectList__rosidl_typesupport_introspection_c__assign_function__ObjectList__objects(
  void * untyped_member, size_t index, const void * untyped_value)
{
  autonav_interfaces__msg__Object * item =
    ((autonav_interfaces__msg__Object *)
    autonav_interfaces__msg__ObjectList__rosidl_typesupport_introspection_c__get_function__ObjectList__objects(untyped_member, index));
  const autonav_interfaces__msg__Object * value =
    (const autonav_interfaces__msg__Object *)(untyped_value);
  *item = *value;
}

bool autonav_interfaces__msg__ObjectList__rosidl_typesupport_introspection_c__resize_function__ObjectList__objects(
  void * untyped_member, size_t size)
{
  autonav_interfaces__msg__Object__Sequence * member =
    (autonav_interfaces__msg__Object__Sequence *)(untyped_member);
  autonav_interfaces__msg__Object__Sequence__fini(member);
  return autonav_interfaces__msg__Object__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember autonav_interfaces__msg__ObjectList__rosidl_typesupport_introspection_c__ObjectList_message_member_array[1] = {
  {
    "objects",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autonav_interfaces__msg__ObjectList, objects),  // bytes offset in struct
    NULL,  // default value
    autonav_interfaces__msg__ObjectList__rosidl_typesupport_introspection_c__size_function__ObjectList__objects,  // size() function pointer
    autonav_interfaces__msg__ObjectList__rosidl_typesupport_introspection_c__get_const_function__ObjectList__objects,  // get_const(index) function pointer
    autonav_interfaces__msg__ObjectList__rosidl_typesupport_introspection_c__get_function__ObjectList__objects,  // get(index) function pointer
    autonav_interfaces__msg__ObjectList__rosidl_typesupport_introspection_c__fetch_function__ObjectList__objects,  // fetch(index, &value) function pointer
    autonav_interfaces__msg__ObjectList__rosidl_typesupport_introspection_c__assign_function__ObjectList__objects,  // assign(index, value) function pointer
    autonav_interfaces__msg__ObjectList__rosidl_typesupport_introspection_c__resize_function__ObjectList__objects  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autonav_interfaces__msg__ObjectList__rosidl_typesupport_introspection_c__ObjectList_message_members = {
  "autonav_interfaces__msg",  // message namespace
  "ObjectList",  // message name
  1,  // number of fields
  sizeof(autonav_interfaces__msg__ObjectList),
  autonav_interfaces__msg__ObjectList__rosidl_typesupport_introspection_c__ObjectList_message_member_array,  // message members
  autonav_interfaces__msg__ObjectList__rosidl_typesupport_introspection_c__ObjectList_init_function,  // function to initialize message memory (memory has to be allocated)
  autonav_interfaces__msg__ObjectList__rosidl_typesupport_introspection_c__ObjectList_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autonav_interfaces__msg__ObjectList__rosidl_typesupport_introspection_c__ObjectList_message_type_support_handle = {
  0,
  &autonav_interfaces__msg__ObjectList__rosidl_typesupport_introspection_c__ObjectList_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autonav_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonav_interfaces, msg, ObjectList)() {
  autonav_interfaces__msg__ObjectList__rosidl_typesupport_introspection_c__ObjectList_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autonav_interfaces, msg, Object)();
  if (!autonav_interfaces__msg__ObjectList__rosidl_typesupport_introspection_c__ObjectList_message_type_support_handle.typesupport_identifier) {
    autonav_interfaces__msg__ObjectList__rosidl_typesupport_introspection_c__ObjectList_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autonav_interfaces__msg__ObjectList__rosidl_typesupport_introspection_c__ObjectList_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
