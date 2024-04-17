// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from roboclaw_ros2:msg/RoboclawEncoderSteps.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "roboclaw_ros2/msg/detail/roboclaw_encoder_steps__rosidl_typesupport_introspection_c.h"
#include "roboclaw_ros2/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "roboclaw_ros2/msg/detail/roboclaw_encoder_steps__functions.h"
#include "roboclaw_ros2/msg/detail/roboclaw_encoder_steps__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void roboclaw_ros2__msg__RoboclawEncoderSteps__rosidl_typesupport_introspection_c__RoboclawEncoderSteps_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  roboclaw_ros2__msg__RoboclawEncoderSteps__init(message_memory);
}

void roboclaw_ros2__msg__RoboclawEncoderSteps__rosidl_typesupport_introspection_c__RoboclawEncoderSteps_fini_function(void * message_memory)
{
  roboclaw_ros2__msg__RoboclawEncoderSteps__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember roboclaw_ros2__msg__RoboclawEncoderSteps__rosidl_typesupport_introspection_c__RoboclawEncoderSteps_message_member_array[3] = {
  {
    "index",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roboclaw_ros2__msg__RoboclawEncoderSteps, index),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "mot1_enc_steps",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roboclaw_ros2__msg__RoboclawEncoderSteps, mot1_enc_steps),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "mot2_enc_steps",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roboclaw_ros2__msg__RoboclawEncoderSteps, mot2_enc_steps),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers roboclaw_ros2__msg__RoboclawEncoderSteps__rosidl_typesupport_introspection_c__RoboclawEncoderSteps_message_members = {
  "roboclaw_ros2__msg",  // message namespace
  "RoboclawEncoderSteps",  // message name
  3,  // number of fields
  sizeof(roboclaw_ros2__msg__RoboclawEncoderSteps),
  roboclaw_ros2__msg__RoboclawEncoderSteps__rosidl_typesupport_introspection_c__RoboclawEncoderSteps_message_member_array,  // message members
  roboclaw_ros2__msg__RoboclawEncoderSteps__rosidl_typesupport_introspection_c__RoboclawEncoderSteps_init_function,  // function to initialize message memory (memory has to be allocated)
  roboclaw_ros2__msg__RoboclawEncoderSteps__rosidl_typesupport_introspection_c__RoboclawEncoderSteps_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t roboclaw_ros2__msg__RoboclawEncoderSteps__rosidl_typesupport_introspection_c__RoboclawEncoderSteps_message_type_support_handle = {
  0,
  &roboclaw_ros2__msg__RoboclawEncoderSteps__rosidl_typesupport_introspection_c__RoboclawEncoderSteps_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_roboclaw_ros2
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, roboclaw_ros2, msg, RoboclawEncoderSteps)() {
  if (!roboclaw_ros2__msg__RoboclawEncoderSteps__rosidl_typesupport_introspection_c__RoboclawEncoderSteps_message_type_support_handle.typesupport_identifier) {
    roboclaw_ros2__msg__RoboclawEncoderSteps__rosidl_typesupport_introspection_c__RoboclawEncoderSteps_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &roboclaw_ros2__msg__RoboclawEncoderSteps__rosidl_typesupport_introspection_c__RoboclawEncoderSteps_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
