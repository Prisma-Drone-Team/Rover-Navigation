// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from roboclaw_ros2:msg/RoboclawEncoderSteps.idl
// generated code does not contain a copyright notice

#ifndef ROBOCLAW_ROS2__MSG__DETAIL__ROBOCLAW_ENCODER_STEPS__BUILDER_HPP_
#define ROBOCLAW_ROS2__MSG__DETAIL__ROBOCLAW_ENCODER_STEPS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "roboclaw_ros2/msg/detail/roboclaw_encoder_steps__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace roboclaw_ros2
{

namespace msg
{

namespace builder
{

class Init_RoboclawEncoderSteps_mot2_enc_steps
{
public:
  explicit Init_RoboclawEncoderSteps_mot2_enc_steps(::roboclaw_ros2::msg::RoboclawEncoderSteps & msg)
  : msg_(msg)
  {}
  ::roboclaw_ros2::msg::RoboclawEncoderSteps mot2_enc_steps(::roboclaw_ros2::msg::RoboclawEncoderSteps::_mot2_enc_steps_type arg)
  {
    msg_.mot2_enc_steps = std::move(arg);
    return std::move(msg_);
  }

private:
  ::roboclaw_ros2::msg::RoboclawEncoderSteps msg_;
};

class Init_RoboclawEncoderSteps_mot1_enc_steps
{
public:
  explicit Init_RoboclawEncoderSteps_mot1_enc_steps(::roboclaw_ros2::msg::RoboclawEncoderSteps & msg)
  : msg_(msg)
  {}
  Init_RoboclawEncoderSteps_mot2_enc_steps mot1_enc_steps(::roboclaw_ros2::msg::RoboclawEncoderSteps::_mot1_enc_steps_type arg)
  {
    msg_.mot1_enc_steps = std::move(arg);
    return Init_RoboclawEncoderSteps_mot2_enc_steps(msg_);
  }

private:
  ::roboclaw_ros2::msg::RoboclawEncoderSteps msg_;
};

class Init_RoboclawEncoderSteps_index
{
public:
  Init_RoboclawEncoderSteps_index()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RoboclawEncoderSteps_mot1_enc_steps index(::roboclaw_ros2::msg::RoboclawEncoderSteps::_index_type arg)
  {
    msg_.index = std::move(arg);
    return Init_RoboclawEncoderSteps_mot1_enc_steps(msg_);
  }

private:
  ::roboclaw_ros2::msg::RoboclawEncoderSteps msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::roboclaw_ros2::msg::RoboclawEncoderSteps>()
{
  return roboclaw_ros2::msg::builder::Init_RoboclawEncoderSteps_index();
}

}  // namespace roboclaw_ros2

#endif  // ROBOCLAW_ROS2__MSG__DETAIL__ROBOCLAW_ENCODER_STEPS__BUILDER_HPP_
