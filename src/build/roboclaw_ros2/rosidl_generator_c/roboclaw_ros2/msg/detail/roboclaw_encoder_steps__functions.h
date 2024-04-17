// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from roboclaw_ros2:msg/RoboclawEncoderSteps.idl
// generated code does not contain a copyright notice

#ifndef ROBOCLAW_ROS2__MSG__DETAIL__ROBOCLAW_ENCODER_STEPS__FUNCTIONS_H_
#define ROBOCLAW_ROS2__MSG__DETAIL__ROBOCLAW_ENCODER_STEPS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "roboclaw_ros2/msg/rosidl_generator_c__visibility_control.h"

#include "roboclaw_ros2/msg/detail/roboclaw_encoder_steps__struct.h"

/// Initialize msg/RoboclawEncoderSteps message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * roboclaw_ros2__msg__RoboclawEncoderSteps
 * )) before or use
 * roboclaw_ros2__msg__RoboclawEncoderSteps__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_roboclaw_ros2
bool
roboclaw_ros2__msg__RoboclawEncoderSteps__init(roboclaw_ros2__msg__RoboclawEncoderSteps * msg);

/// Finalize msg/RoboclawEncoderSteps message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_roboclaw_ros2
void
roboclaw_ros2__msg__RoboclawEncoderSteps__fini(roboclaw_ros2__msg__RoboclawEncoderSteps * msg);

/// Create msg/RoboclawEncoderSteps message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * roboclaw_ros2__msg__RoboclawEncoderSteps__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_roboclaw_ros2
roboclaw_ros2__msg__RoboclawEncoderSteps *
roboclaw_ros2__msg__RoboclawEncoderSteps__create();

/// Destroy msg/RoboclawEncoderSteps message.
/**
 * It calls
 * roboclaw_ros2__msg__RoboclawEncoderSteps__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_roboclaw_ros2
void
roboclaw_ros2__msg__RoboclawEncoderSteps__destroy(roboclaw_ros2__msg__RoboclawEncoderSteps * msg);

/// Check for msg/RoboclawEncoderSteps message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_roboclaw_ros2
bool
roboclaw_ros2__msg__RoboclawEncoderSteps__are_equal(const roboclaw_ros2__msg__RoboclawEncoderSteps * lhs, const roboclaw_ros2__msg__RoboclawEncoderSteps * rhs);

/// Copy a msg/RoboclawEncoderSteps message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_roboclaw_ros2
bool
roboclaw_ros2__msg__RoboclawEncoderSteps__copy(
  const roboclaw_ros2__msg__RoboclawEncoderSteps * input,
  roboclaw_ros2__msg__RoboclawEncoderSteps * output);

/// Initialize array of msg/RoboclawEncoderSteps messages.
/**
 * It allocates the memory for the number of elements and calls
 * roboclaw_ros2__msg__RoboclawEncoderSteps__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_roboclaw_ros2
bool
roboclaw_ros2__msg__RoboclawEncoderSteps__Sequence__init(roboclaw_ros2__msg__RoboclawEncoderSteps__Sequence * array, size_t size);

/// Finalize array of msg/RoboclawEncoderSteps messages.
/**
 * It calls
 * roboclaw_ros2__msg__RoboclawEncoderSteps__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_roboclaw_ros2
void
roboclaw_ros2__msg__RoboclawEncoderSteps__Sequence__fini(roboclaw_ros2__msg__RoboclawEncoderSteps__Sequence * array);

/// Create array of msg/RoboclawEncoderSteps messages.
/**
 * It allocates the memory for the array and calls
 * roboclaw_ros2__msg__RoboclawEncoderSteps__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_roboclaw_ros2
roboclaw_ros2__msg__RoboclawEncoderSteps__Sequence *
roboclaw_ros2__msg__RoboclawEncoderSteps__Sequence__create(size_t size);

/// Destroy array of msg/RoboclawEncoderSteps messages.
/**
 * It calls
 * roboclaw_ros2__msg__RoboclawEncoderSteps__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_roboclaw_ros2
void
roboclaw_ros2__msg__RoboclawEncoderSteps__Sequence__destroy(roboclaw_ros2__msg__RoboclawEncoderSteps__Sequence * array);

/// Check for msg/RoboclawEncoderSteps message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_roboclaw_ros2
bool
roboclaw_ros2__msg__RoboclawEncoderSteps__Sequence__are_equal(const roboclaw_ros2__msg__RoboclawEncoderSteps__Sequence * lhs, const roboclaw_ros2__msg__RoboclawEncoderSteps__Sequence * rhs);

/// Copy an array of msg/RoboclawEncoderSteps messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_roboclaw_ros2
bool
roboclaw_ros2__msg__RoboclawEncoderSteps__Sequence__copy(
  const roboclaw_ros2__msg__RoboclawEncoderSteps__Sequence * input,
  roboclaw_ros2__msg__RoboclawEncoderSteps__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ROBOCLAW_ROS2__MSG__DETAIL__ROBOCLAW_ENCODER_STEPS__FUNCTIONS_H_
