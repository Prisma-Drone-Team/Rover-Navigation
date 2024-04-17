// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from roboclaw_ros2:msg/RoboclawMotorVelocity.idl
// generated code does not contain a copyright notice

#ifndef ROBOCLAW_ROS2__MSG__DETAIL__ROBOCLAW_MOTOR_VELOCITY__FUNCTIONS_H_
#define ROBOCLAW_ROS2__MSG__DETAIL__ROBOCLAW_MOTOR_VELOCITY__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "roboclaw_ros2/msg/rosidl_generator_c__visibility_control.h"

#include "roboclaw_ros2/msg/detail/roboclaw_motor_velocity__struct.h"

/// Initialize msg/RoboclawMotorVelocity message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * roboclaw_ros2__msg__RoboclawMotorVelocity
 * )) before or use
 * roboclaw_ros2__msg__RoboclawMotorVelocity__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_roboclaw_ros2
bool
roboclaw_ros2__msg__RoboclawMotorVelocity__init(roboclaw_ros2__msg__RoboclawMotorVelocity * msg);

/// Finalize msg/RoboclawMotorVelocity message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_roboclaw_ros2
void
roboclaw_ros2__msg__RoboclawMotorVelocity__fini(roboclaw_ros2__msg__RoboclawMotorVelocity * msg);

/// Create msg/RoboclawMotorVelocity message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * roboclaw_ros2__msg__RoboclawMotorVelocity__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_roboclaw_ros2
roboclaw_ros2__msg__RoboclawMotorVelocity *
roboclaw_ros2__msg__RoboclawMotorVelocity__create();

/// Destroy msg/RoboclawMotorVelocity message.
/**
 * It calls
 * roboclaw_ros2__msg__RoboclawMotorVelocity__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_roboclaw_ros2
void
roboclaw_ros2__msg__RoboclawMotorVelocity__destroy(roboclaw_ros2__msg__RoboclawMotorVelocity * msg);

/// Check for msg/RoboclawMotorVelocity message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_roboclaw_ros2
bool
roboclaw_ros2__msg__RoboclawMotorVelocity__are_equal(const roboclaw_ros2__msg__RoboclawMotorVelocity * lhs, const roboclaw_ros2__msg__RoboclawMotorVelocity * rhs);

/// Copy a msg/RoboclawMotorVelocity message.
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
roboclaw_ros2__msg__RoboclawMotorVelocity__copy(
  const roboclaw_ros2__msg__RoboclawMotorVelocity * input,
  roboclaw_ros2__msg__RoboclawMotorVelocity * output);

/// Initialize array of msg/RoboclawMotorVelocity messages.
/**
 * It allocates the memory for the number of elements and calls
 * roboclaw_ros2__msg__RoboclawMotorVelocity__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_roboclaw_ros2
bool
roboclaw_ros2__msg__RoboclawMotorVelocity__Sequence__init(roboclaw_ros2__msg__RoboclawMotorVelocity__Sequence * array, size_t size);

/// Finalize array of msg/RoboclawMotorVelocity messages.
/**
 * It calls
 * roboclaw_ros2__msg__RoboclawMotorVelocity__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_roboclaw_ros2
void
roboclaw_ros2__msg__RoboclawMotorVelocity__Sequence__fini(roboclaw_ros2__msg__RoboclawMotorVelocity__Sequence * array);

/// Create array of msg/RoboclawMotorVelocity messages.
/**
 * It allocates the memory for the array and calls
 * roboclaw_ros2__msg__RoboclawMotorVelocity__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_roboclaw_ros2
roboclaw_ros2__msg__RoboclawMotorVelocity__Sequence *
roboclaw_ros2__msg__RoboclawMotorVelocity__Sequence__create(size_t size);

/// Destroy array of msg/RoboclawMotorVelocity messages.
/**
 * It calls
 * roboclaw_ros2__msg__RoboclawMotorVelocity__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_roboclaw_ros2
void
roboclaw_ros2__msg__RoboclawMotorVelocity__Sequence__destroy(roboclaw_ros2__msg__RoboclawMotorVelocity__Sequence * array);

/// Check for msg/RoboclawMotorVelocity message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_roboclaw_ros2
bool
roboclaw_ros2__msg__RoboclawMotorVelocity__Sequence__are_equal(const roboclaw_ros2__msg__RoboclawMotorVelocity__Sequence * lhs, const roboclaw_ros2__msg__RoboclawMotorVelocity__Sequence * rhs);

/// Copy an array of msg/RoboclawMotorVelocity messages.
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
roboclaw_ros2__msg__RoboclawMotorVelocity__Sequence__copy(
  const roboclaw_ros2__msg__RoboclawMotorVelocity__Sequence * input,
  roboclaw_ros2__msg__RoboclawMotorVelocity__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ROBOCLAW_ROS2__MSG__DETAIL__ROBOCLAW_MOTOR_VELOCITY__FUNCTIONS_H_
