// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from roboclaw_ros2:msg/RoboclawMotorVelocity.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "roboclaw_ros2/msg/detail/roboclaw_motor_velocity__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace roboclaw_ros2
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void RoboclawMotorVelocity_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) roboclaw_ros2::msg::RoboclawMotorVelocity(_init);
}

void RoboclawMotorVelocity_fini_function(void * message_memory)
{
  auto typed_message = static_cast<roboclaw_ros2::msg::RoboclawMotorVelocity *>(message_memory);
  typed_message->~RoboclawMotorVelocity();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember RoboclawMotorVelocity_message_member_array[3] = {
  {
    "index",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roboclaw_ros2::msg::RoboclawMotorVelocity, index),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "mot1_vel_sps",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roboclaw_ros2::msg::RoboclawMotorVelocity, mot1_vel_sps),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "mot2_vel_sps",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roboclaw_ros2::msg::RoboclawMotorVelocity, mot2_vel_sps),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers RoboclawMotorVelocity_message_members = {
  "roboclaw_ros2::msg",  // message namespace
  "RoboclawMotorVelocity",  // message name
  3,  // number of fields
  sizeof(roboclaw_ros2::msg::RoboclawMotorVelocity),
  RoboclawMotorVelocity_message_member_array,  // message members
  RoboclawMotorVelocity_init_function,  // function to initialize message memory (memory has to be allocated)
  RoboclawMotorVelocity_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t RoboclawMotorVelocity_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &RoboclawMotorVelocity_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace roboclaw_ros2


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<roboclaw_ros2::msg::RoboclawMotorVelocity>()
{
  return &::roboclaw_ros2::msg::rosidl_typesupport_introspection_cpp::RoboclawMotorVelocity_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, roboclaw_ros2, msg, RoboclawMotorVelocity)() {
  return &::roboclaw_ros2::msg::rosidl_typesupport_introspection_cpp::RoboclawMotorVelocity_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
