// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from roboclaw_ros2:msg/RoboclawMotorVelocity.idl
// generated code does not contain a copyright notice
#include "roboclaw_ros2/msg/detail/roboclaw_motor_velocity__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
roboclaw_ros2__msg__RoboclawMotorVelocity__init(roboclaw_ros2__msg__RoboclawMotorVelocity * msg)
{
  if (!msg) {
    return false;
  }
  // index
  // mot1_vel_sps
  // mot2_vel_sps
  return true;
}

void
roboclaw_ros2__msg__RoboclawMotorVelocity__fini(roboclaw_ros2__msg__RoboclawMotorVelocity * msg)
{
  if (!msg) {
    return;
  }
  // index
  // mot1_vel_sps
  // mot2_vel_sps
}

bool
roboclaw_ros2__msg__RoboclawMotorVelocity__are_equal(const roboclaw_ros2__msg__RoboclawMotorVelocity * lhs, const roboclaw_ros2__msg__RoboclawMotorVelocity * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // index
  if (lhs->index != rhs->index) {
    return false;
  }
  // mot1_vel_sps
  if (lhs->mot1_vel_sps != rhs->mot1_vel_sps) {
    return false;
  }
  // mot2_vel_sps
  if (lhs->mot2_vel_sps != rhs->mot2_vel_sps) {
    return false;
  }
  return true;
}

bool
roboclaw_ros2__msg__RoboclawMotorVelocity__copy(
  const roboclaw_ros2__msg__RoboclawMotorVelocity * input,
  roboclaw_ros2__msg__RoboclawMotorVelocity * output)
{
  if (!input || !output) {
    return false;
  }
  // index
  output->index = input->index;
  // mot1_vel_sps
  output->mot1_vel_sps = input->mot1_vel_sps;
  // mot2_vel_sps
  output->mot2_vel_sps = input->mot2_vel_sps;
  return true;
}

roboclaw_ros2__msg__RoboclawMotorVelocity *
roboclaw_ros2__msg__RoboclawMotorVelocity__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  roboclaw_ros2__msg__RoboclawMotorVelocity * msg = (roboclaw_ros2__msg__RoboclawMotorVelocity *)allocator.allocate(sizeof(roboclaw_ros2__msg__RoboclawMotorVelocity), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(roboclaw_ros2__msg__RoboclawMotorVelocity));
  bool success = roboclaw_ros2__msg__RoboclawMotorVelocity__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
roboclaw_ros2__msg__RoboclawMotorVelocity__destroy(roboclaw_ros2__msg__RoboclawMotorVelocity * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    roboclaw_ros2__msg__RoboclawMotorVelocity__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
roboclaw_ros2__msg__RoboclawMotorVelocity__Sequence__init(roboclaw_ros2__msg__RoboclawMotorVelocity__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  roboclaw_ros2__msg__RoboclawMotorVelocity * data = NULL;

  if (size) {
    data = (roboclaw_ros2__msg__RoboclawMotorVelocity *)allocator.zero_allocate(size, sizeof(roboclaw_ros2__msg__RoboclawMotorVelocity), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = roboclaw_ros2__msg__RoboclawMotorVelocity__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        roboclaw_ros2__msg__RoboclawMotorVelocity__fini(&data[i - 1]);
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
roboclaw_ros2__msg__RoboclawMotorVelocity__Sequence__fini(roboclaw_ros2__msg__RoboclawMotorVelocity__Sequence * array)
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
      roboclaw_ros2__msg__RoboclawMotorVelocity__fini(&array->data[i]);
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

roboclaw_ros2__msg__RoboclawMotorVelocity__Sequence *
roboclaw_ros2__msg__RoboclawMotorVelocity__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  roboclaw_ros2__msg__RoboclawMotorVelocity__Sequence * array = (roboclaw_ros2__msg__RoboclawMotorVelocity__Sequence *)allocator.allocate(sizeof(roboclaw_ros2__msg__RoboclawMotorVelocity__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = roboclaw_ros2__msg__RoboclawMotorVelocity__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
roboclaw_ros2__msg__RoboclawMotorVelocity__Sequence__destroy(roboclaw_ros2__msg__RoboclawMotorVelocity__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    roboclaw_ros2__msg__RoboclawMotorVelocity__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
roboclaw_ros2__msg__RoboclawMotorVelocity__Sequence__are_equal(const roboclaw_ros2__msg__RoboclawMotorVelocity__Sequence * lhs, const roboclaw_ros2__msg__RoboclawMotorVelocity__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!roboclaw_ros2__msg__RoboclawMotorVelocity__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
roboclaw_ros2__msg__RoboclawMotorVelocity__Sequence__copy(
  const roboclaw_ros2__msg__RoboclawMotorVelocity__Sequence * input,
  roboclaw_ros2__msg__RoboclawMotorVelocity__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(roboclaw_ros2__msg__RoboclawMotorVelocity);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    roboclaw_ros2__msg__RoboclawMotorVelocity * data =
      (roboclaw_ros2__msg__RoboclawMotorVelocity *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!roboclaw_ros2__msg__RoboclawMotorVelocity__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          roboclaw_ros2__msg__RoboclawMotorVelocity__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!roboclaw_ros2__msg__RoboclawMotorVelocity__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
