// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autonav_interfaces:msg/Object.idl
// generated code does not contain a copyright notice
#include "autonav_interfaces/msg/detail/object__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
autonav_interfaces__msg__Object__init(autonav_interfaces__msg__Object * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // z
  return true;
}

void
autonav_interfaces__msg__Object__fini(autonav_interfaces__msg__Object * msg)
{
  if (!msg) {
    return;
  }
  // x
  // z
}

bool
autonav_interfaces__msg__Object__are_equal(const autonav_interfaces__msg__Object * lhs, const autonav_interfaces__msg__Object * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // z
  if (lhs->z != rhs->z) {
    return false;
  }
  return true;
}

bool
autonav_interfaces__msg__Object__copy(
  const autonav_interfaces__msg__Object * input,
  autonav_interfaces__msg__Object * output)
{
  if (!input || !output) {
    return false;
  }
  // x
  output->x = input->x;
  // z
  output->z = input->z;
  return true;
}

autonav_interfaces__msg__Object *
autonav_interfaces__msg__Object__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonav_interfaces__msg__Object * msg = (autonav_interfaces__msg__Object *)allocator.allocate(sizeof(autonav_interfaces__msg__Object), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autonav_interfaces__msg__Object));
  bool success = autonav_interfaces__msg__Object__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autonav_interfaces__msg__Object__destroy(autonav_interfaces__msg__Object * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autonav_interfaces__msg__Object__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autonav_interfaces__msg__Object__Sequence__init(autonav_interfaces__msg__Object__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonav_interfaces__msg__Object * data = NULL;

  if (size) {
    data = (autonav_interfaces__msg__Object *)allocator.zero_allocate(size, sizeof(autonav_interfaces__msg__Object), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autonav_interfaces__msg__Object__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autonav_interfaces__msg__Object__fini(&data[i - 1]);
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
autonav_interfaces__msg__Object__Sequence__fini(autonav_interfaces__msg__Object__Sequence * array)
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
      autonav_interfaces__msg__Object__fini(&array->data[i]);
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

autonav_interfaces__msg__Object__Sequence *
autonav_interfaces__msg__Object__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autonav_interfaces__msg__Object__Sequence * array = (autonav_interfaces__msg__Object__Sequence *)allocator.allocate(sizeof(autonav_interfaces__msg__Object__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autonav_interfaces__msg__Object__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autonav_interfaces__msg__Object__Sequence__destroy(autonav_interfaces__msg__Object__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autonav_interfaces__msg__Object__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autonav_interfaces__msg__Object__Sequence__are_equal(const autonav_interfaces__msg__Object__Sequence * lhs, const autonav_interfaces__msg__Object__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autonav_interfaces__msg__Object__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autonav_interfaces__msg__Object__Sequence__copy(
  const autonav_interfaces__msg__Object__Sequence * input,
  autonav_interfaces__msg__Object__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autonav_interfaces__msg__Object);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autonav_interfaces__msg__Object * data =
      (autonav_interfaces__msg__Object *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autonav_interfaces__msg__Object__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autonav_interfaces__msg__Object__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autonav_interfaces__msg__Object__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
