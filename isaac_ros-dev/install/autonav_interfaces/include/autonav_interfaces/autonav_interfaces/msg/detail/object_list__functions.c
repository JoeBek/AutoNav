// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autonav_interfaces:msg/ObjectList.idl
// generated code does not contain a copyright notice
#include "autonav_interfaces/msg/detail/object_list__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `objects`
#include "autonav_interfaces/msg/detail/object__functions.h"

bool
autonav_interfaces__msg__ObjectList__init(autonav_interfaces__msg__ObjectList * msg)
{
  if (!msg) {
    return false;
  }
  // objects
  if (!autonav_interfaces__msg__Object__Sequence__init(&msg->objects, 0)) {
    autonav_interfaces__msg__ObjectList__fini(msg);
    return false;
  }
  return true;
}

void
autonav_interfaces__msg__ObjectList__fini(autonav_interfaces__msg__ObjectList * msg)
{
  if (!msg) {
    return;
  }
  // objects
  autonav_interfaces__msg__Object__Sequence__fini(&msg->objects);
}

bool
autonav_interfaces__msg__ObjectList__are_equal(const autonav_interfaces__msg__ObjectList * lhs, const autonav_interfaces__msg__ObjectList * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // objects
  if (!autonav_interfaces__msg__Object__Sequence__are_equal(
      &(lhs->objects), &(rhs->objects)))
  {
    return false;
  }
  return true;
}

bool
autonav_interfaces__msg__ObjectList__copy(
  const autonav_interfaces__msg__ObjectList * input,
  autonav_interfaces__msg__ObjectList * output)
{
  if (!input || !output) {
    return false;
  }
  // objects
  if (!autonav_interfaces__msg__Object__Sequence__copy(
      &(input->objects), &(output->objects)))
  {
    return false;
  }
  return true;
}

autonav_interfaces__msg__ObjectList *
autonav_interfaces__msg__ObjectList__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonav_interfaces__msg__ObjectList * msg = (autonav_interfaces__msg__ObjectList *)allocator.allocate(sizeof(autonav_interfaces__msg__ObjectList), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonav_interfaces__msg__ObjectList));
  bool success = autonav_interfaces__msg__ObjectList__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonav_interfaces__msg__ObjectList__destroy(autonav_interfaces__msg__ObjectList * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonav_interfaces__msg__ObjectList__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonav_interfaces__msg__ObjectList__Sequence__init(autonav_interfaces__msg__ObjectList__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonav_interfaces__msg__ObjectList * data = NULL;

  if (size) {
    data = (autonav_interfaces__msg__ObjectList *)allocator.zero_allocate(size, sizeof(autonav_interfaces__msg__ObjectList), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonav_interfaces__msg__ObjectList__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonav_interfaces__msg__ObjectList__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
autonav_interfaces__msg__ObjectList__Sequence__fini(autonav_interfaces__msg__ObjectList__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      autonav_interfaces__msg__ObjectList__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

autonav_interfaces__msg__ObjectList__Sequence *
autonav_interfaces__msg__ObjectList__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonav_interfaces__msg__ObjectList__Sequence * array = (autonav_interfaces__msg__ObjectList__Sequence *)allocator.allocate(sizeof(autonav_interfaces__msg__ObjectList__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonav_interfaces__msg__ObjectList__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonav_interfaces__msg__ObjectList__Sequence__destroy(autonav_interfaces__msg__ObjectList__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonav_interfaces__msg__ObjectList__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonav_interfaces__msg__ObjectList__Sequence__are_equal(const autonav_interfaces__msg__ObjectList__Sequence * lhs, const autonav_interfaces__msg__ObjectList__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonav_interfaces__msg__ObjectList__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonav_interfaces__msg__ObjectList__Sequence__copy(
  const autonav_interfaces__msg__ObjectList__Sequence * input,
  autonav_interfaces__msg__ObjectList__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonav_interfaces__msg__ObjectList);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonav_interfaces__msg__ObjectList * data =
      (autonav_interfaces__msg__ObjectList *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonav_interfaces__msg__ObjectList__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonav_interfaces__msg__ObjectList__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonav_interfaces__msg__ObjectList__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
