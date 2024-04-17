// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from roboclaw_ros2:msg/RoboclawEncoderSteps.idl
// generated code does not contain a copyright notice
#include "roboclaw_ros2/msg/detail/roboclaw_encoder_steps__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
roboclaw_ros2__msg__RoboclawEncoderSteps__init(roboclaw_ros2__msg__RoboclawEncoderSteps * msg)
{
  if (!msg) {
    return false;
  }
  // index
  // mot1_enc_steps
  // mot2_enc_steps
  return true;
}

void
roboclaw_ros2__msg__RoboclawEncoderSteps__fini(roboclaw_ros2__msg__RoboclawEncoderSteps * msg)
{
  if (!msg) {
    return;
  }
  // index
  // mot1_enc_steps
  // mot2_enc_steps
}

bool
roboclaw_ros2__msg__RoboclawEncoderSteps__are_equal(const roboclaw_ros2__msg__RoboclawEncoderSteps * lhs, const roboclaw_ros2__msg__RoboclawEncoderSteps * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // index
  if (lhs->index != rhs->index) {
    return false;
  }
  // mot1_enc_steps
  if (lhs->mot1_enc_steps != rhs->mot1_enc_steps) {
    return false;
  }
  // mot2_enc_steps
  if (lhs->mot2_enc_steps != rhs->mot2_enc_steps) {
    return false;
  }
  return true;
}

bool
roboclaw_ros2__msg__RoboclawEncoderSteps__copy(
  const roboclaw_ros2__msg__RoboclawEncoderSteps * input,
  roboclaw_ros2__msg__RoboclawEncoderSteps * output)
{
  if (!input || !output) {
    return false;
  }
  // index
  output->index = input->index;
  // mot1_enc_steps
  output->mot1_enc_steps = input->mot1_enc_steps;
  // mot2_enc_steps
  output->mot2_enc_steps = input->mot2_enc_steps;
  return true;
}

roboclaw_ros2__msg__RoboclawEncoderSteps *
roboclaw_ros2__msg__RoboclawEncoderSteps__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  roboclaw_ros2__msg__RoboclawEncoderSteps * msg = (roboclaw_ros2__msg__RoboclawEncoderSteps *)allocator.allocate(sizeof(roboclaw_ros2__msg__RoboclawEncoderSteps), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(roboclaw_ros2__msg__RoboclawEncoderSteps));
  bool success = roboclaw_ros2__msg__RoboclawEncoderSteps__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
roboclaw_ros2__msg__RoboclawEncoderSteps__destroy(roboclaw_ros2__msg__RoboclawEncoderSteps * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    roboclaw_ros2__msg__RoboclawEncoderSteps__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
roboclaw_ros2__msg__RoboclawEncoderSteps__Sequence__init(roboclaw_ros2__msg__RoboclawEncoderSteps__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  roboclaw_ros2__msg__RoboclawEncoderSteps * data = NULL;

  if (size) {
    data = (roboclaw_ros2__msg__RoboclawEncoderSteps *)allocator.zero_allocate(size, sizeof(roboclaw_ros2__msg__RoboclawEncoderSteps), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = roboclaw_ros2__msg__RoboclawEncoderSteps__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        roboclaw_ros2__msg__RoboclawEncoderSteps__fini(&data[i - 1]);
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
roboclaw_ros2__msg__RoboclawEncoderSteps__Sequence__fini(roboclaw_ros2__msg__RoboclawEncoderSteps__Sequence * array)
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
      roboclaw_ros2__msg__RoboclawEncoderSteps__fini(&array->data[i]);
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

roboclaw_ros2__msg__RoboclawEncoderSteps__Sequence *
roboclaw_ros2__msg__RoboclawEncoderSteps__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  roboclaw_ros2__msg__RoboclawEncoderSteps__Sequence * array = (roboclaw_ros2__msg__RoboclawEncoderSteps__Sequence *)allocator.allocate(sizeof(roboclaw_ros2__msg__RoboclawEncoderSteps__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = roboclaw_ros2__msg__RoboclawEncoderSteps__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
roboclaw_ros2__msg__RoboclawEncoderSteps__Sequence__destroy(roboclaw_ros2__msg__RoboclawEncoderSteps__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    roboclaw_ros2__msg__RoboclawEncoderSteps__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
roboclaw_ros2__msg__RoboclawEncoderSteps__Sequence__are_equal(const roboclaw_ros2__msg__RoboclawEncoderSteps__Sequence * lhs, const roboclaw_ros2__msg__RoboclawEncoderSteps__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!roboclaw_ros2__msg__RoboclawEncoderSteps__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
roboclaw_ros2__msg__RoboclawEncoderSteps__Sequence__copy(
  const roboclaw_ros2__msg__RoboclawEncoderSteps__Sequence * input,
  roboclaw_ros2__msg__RoboclawEncoderSteps__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(roboclaw_ros2__msg__RoboclawEncoderSteps);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    roboclaw_ros2__msg__RoboclawEncoderSteps * data =
      (roboclaw_ros2__msg__RoboclawEncoderSteps *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!roboclaw_ros2__msg__RoboclawEncoderSteps__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          roboclaw_ros2__msg__RoboclawEncoderSteps__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!roboclaw_ros2__msg__RoboclawEncoderSteps__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
