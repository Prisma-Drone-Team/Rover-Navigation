// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from roboclaw_ros2:msg/RoboclawEncoderSteps.idl
// generated code does not contain a copyright notice
#include "roboclaw_ros2/msg/detail/roboclaw_encoder_steps__rosidl_typesupport_fastrtps_cpp.hpp"
#include "roboclaw_ros2/msg/detail/roboclaw_encoder_steps__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace roboclaw_ros2
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_roboclaw_ros2
cdr_serialize(
  const roboclaw_ros2::msg::RoboclawEncoderSteps & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: index
  cdr << ros_message.index;
  // Member: mot1_enc_steps
  cdr << ros_message.mot1_enc_steps;
  // Member: mot2_enc_steps
  cdr << ros_message.mot2_enc_steps;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_roboclaw_ros2
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  roboclaw_ros2::msg::RoboclawEncoderSteps & ros_message)
{
  // Member: index
  cdr >> ros_message.index;

  // Member: mot1_enc_steps
  cdr >> ros_message.mot1_enc_steps;

  // Member: mot2_enc_steps
  cdr >> ros_message.mot2_enc_steps;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_roboclaw_ros2
get_serialized_size(
  const roboclaw_ros2::msg::RoboclawEncoderSteps & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: index
  {
    size_t item_size = sizeof(ros_message.index);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: mot1_enc_steps
  {
    size_t item_size = sizeof(ros_message.mot1_enc_steps);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: mot2_enc_steps
  {
    size_t item_size = sizeof(ros_message.mot2_enc_steps);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_roboclaw_ros2
max_serialized_size_RoboclawEncoderSteps(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: index
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: mot1_enc_steps
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: mot2_enc_steps
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = roboclaw_ros2::msg::RoboclawEncoderSteps;
    is_plain =
      (
      offsetof(DataType, mot2_enc_steps) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _RoboclawEncoderSteps__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const roboclaw_ros2::msg::RoboclawEncoderSteps *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _RoboclawEncoderSteps__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<roboclaw_ros2::msg::RoboclawEncoderSteps *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _RoboclawEncoderSteps__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const roboclaw_ros2::msg::RoboclawEncoderSteps *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _RoboclawEncoderSteps__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_RoboclawEncoderSteps(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _RoboclawEncoderSteps__callbacks = {
  "roboclaw_ros2::msg",
  "RoboclawEncoderSteps",
  _RoboclawEncoderSteps__cdr_serialize,
  _RoboclawEncoderSteps__cdr_deserialize,
  _RoboclawEncoderSteps__get_serialized_size,
  _RoboclawEncoderSteps__max_serialized_size
};

static rosidl_message_type_support_t _RoboclawEncoderSteps__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_RoboclawEncoderSteps__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace roboclaw_ros2

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_roboclaw_ros2
const rosidl_message_type_support_t *
get_message_type_support_handle<roboclaw_ros2::msg::RoboclawEncoderSteps>()
{
  return &roboclaw_ros2::msg::typesupport_fastrtps_cpp::_RoboclawEncoderSteps__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, roboclaw_ros2, msg, RoboclawEncoderSteps)() {
  return &roboclaw_ros2::msg::typesupport_fastrtps_cpp::_RoboclawEncoderSteps__handle;
}

#ifdef __cplusplus
}
#endif
