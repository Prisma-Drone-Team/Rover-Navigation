// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from roboclaw_ros2:msg/RoboclawEncoderSteps.idl
// generated code does not contain a copyright notice

#ifndef ROBOCLAW_ROS2__MSG__DETAIL__ROBOCLAW_ENCODER_STEPS__STRUCT_H_
#define ROBOCLAW_ROS2__MSG__DETAIL__ROBOCLAW_ENCODER_STEPS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/RoboclawEncoderSteps in the package roboclaw_ros2.
typedef struct roboclaw_ros2__msg__RoboclawEncoderSteps
{
  int32_t index;
  int32_t mot1_enc_steps;
  int32_t mot2_enc_steps;
} roboclaw_ros2__msg__RoboclawEncoderSteps;

// Struct for a sequence of roboclaw_ros2__msg__RoboclawEncoderSteps.
typedef struct roboclaw_ros2__msg__RoboclawEncoderSteps__Sequence
{
  roboclaw_ros2__msg__RoboclawEncoderSteps * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} roboclaw_ros2__msg__RoboclawEncoderSteps__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOCLAW_ROS2__MSG__DETAIL__ROBOCLAW_ENCODER_STEPS__STRUCT_H_
