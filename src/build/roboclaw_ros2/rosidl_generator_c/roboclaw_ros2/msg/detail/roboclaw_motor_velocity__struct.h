// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from roboclaw_ros2:msg/RoboclawMotorVelocity.idl
// generated code does not contain a copyright notice

#ifndef ROBOCLAW_ROS2__MSG__DETAIL__ROBOCLAW_MOTOR_VELOCITY__STRUCT_H_
#define ROBOCLAW_ROS2__MSG__DETAIL__ROBOCLAW_MOTOR_VELOCITY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/RoboclawMotorVelocity in the package roboclaw_ros2.
typedef struct roboclaw_ros2__msg__RoboclawMotorVelocity
{
  int32_t index;
  int32_t mot1_vel_sps;
  int32_t mot2_vel_sps;
} roboclaw_ros2__msg__RoboclawMotorVelocity;

// Struct for a sequence of roboclaw_ros2__msg__RoboclawMotorVelocity.
typedef struct roboclaw_ros2__msg__RoboclawMotorVelocity__Sequence
{
  roboclaw_ros2__msg__RoboclawMotorVelocity * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} roboclaw_ros2__msg__RoboclawMotorVelocity__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOCLAW_ROS2__MSG__DETAIL__ROBOCLAW_MOTOR_VELOCITY__STRUCT_H_
