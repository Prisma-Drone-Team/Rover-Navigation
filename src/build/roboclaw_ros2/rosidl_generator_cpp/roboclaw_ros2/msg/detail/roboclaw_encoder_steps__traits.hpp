// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from roboclaw_ros2:msg/RoboclawEncoderSteps.idl
// generated code does not contain a copyright notice

#ifndef ROBOCLAW_ROS2__MSG__DETAIL__ROBOCLAW_ENCODER_STEPS__TRAITS_HPP_
#define ROBOCLAW_ROS2__MSG__DETAIL__ROBOCLAW_ENCODER_STEPS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "roboclaw_ros2/msg/detail/roboclaw_encoder_steps__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace roboclaw_ros2
{

namespace msg
{

inline void to_flow_style_yaml(
  const RoboclawEncoderSteps & msg,
  std::ostream & out)
{
  out << "{";
  // member: index
  {
    out << "index: ";
    rosidl_generator_traits::value_to_yaml(msg.index, out);
    out << ", ";
  }

  // member: mot1_enc_steps
  {
    out << "mot1_enc_steps: ";
    rosidl_generator_traits::value_to_yaml(msg.mot1_enc_steps, out);
    out << ", ";
  }

  // member: mot2_enc_steps
  {
    out << "mot2_enc_steps: ";
    rosidl_generator_traits::value_to_yaml(msg.mot2_enc_steps, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RoboclawEncoderSteps & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: index
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "index: ";
    rosidl_generator_traits::value_to_yaml(msg.index, out);
    out << "\n";
  }

  // member: mot1_enc_steps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mot1_enc_steps: ";
    rosidl_generator_traits::value_to_yaml(msg.mot1_enc_steps, out);
    out << "\n";
  }

  // member: mot2_enc_steps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mot2_enc_steps: ";
    rosidl_generator_traits::value_to_yaml(msg.mot2_enc_steps, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RoboclawEncoderSteps & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace roboclaw_ros2

namespace rosidl_generator_traits
{

[[deprecated("use roboclaw_ros2::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const roboclaw_ros2::msg::RoboclawEncoderSteps & msg,
  std::ostream & out, size_t indentation = 0)
{
  roboclaw_ros2::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use roboclaw_ros2::msg::to_yaml() instead")]]
inline std::string to_yaml(const roboclaw_ros2::msg::RoboclawEncoderSteps & msg)
{
  return roboclaw_ros2::msg::to_yaml(msg);
}

template<>
inline const char * data_type<roboclaw_ros2::msg::RoboclawEncoderSteps>()
{
  return "roboclaw_ros2::msg::RoboclawEncoderSteps";
}

template<>
inline const char * name<roboclaw_ros2::msg::RoboclawEncoderSteps>()
{
  return "roboclaw_ros2/msg/RoboclawEncoderSteps";
}

template<>
struct has_fixed_size<roboclaw_ros2::msg::RoboclawEncoderSteps>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<roboclaw_ros2::msg::RoboclawEncoderSteps>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<roboclaw_ros2::msg::RoboclawEncoderSteps>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOCLAW_ROS2__MSG__DETAIL__ROBOCLAW_ENCODER_STEPS__TRAITS_HPP_
