// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from roboclaw_ros2:msg/RoboclawMotorVelocity.idl
// generated code does not contain a copyright notice

#ifndef ROBOCLAW_ROS2__MSG__DETAIL__ROBOCLAW_MOTOR_VELOCITY__BUILDER_HPP_
#define ROBOCLAW_ROS2__MSG__DETAIL__ROBOCLAW_MOTOR_VELOCITY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "roboclaw_ros2/msg/detail/roboclaw_motor_velocity__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace roboclaw_ros2
{

namespace msg
{

namespace builder
{

class Init_RoboclawMotorVelocity_mot2_vel_sps
{
public:
  explicit Init_RoboclawMotorVelocity_mot2_vel_sps(::roboclaw_ros2::msg::RoboclawMotorVelocity & msg)
  : msg_(msg)
  {}
  ::roboclaw_ros2::msg::RoboclawMotorVelocity mot2_vel_sps(::roboclaw_ros2::msg::RoboclawMotorVelocity::_mot2_vel_sps_type arg)
  {
    msg_.mot2_vel_sps = std::move(arg);
    return std::move(msg_);
  }

private:
  ::roboclaw_ros2::msg::RoboclawMotorVelocity msg_;
};

class Init_RoboclawMotorVelocity_mot1_vel_sps
{
public:
  explicit Init_RoboclawMotorVelocity_mot1_vel_sps(::roboclaw_ros2::msg::RoboclawMotorVelocity & msg)
  : msg_(msg)
  {}
  Init_RoboclawMotorVelocity_mot2_vel_sps mot1_vel_sps(::roboclaw_ros2::msg::RoboclawMotorVelocity::_mot1_vel_sps_type arg)
  {
    msg_.mot1_vel_sps = std::move(arg);
    return Init_RoboclawMotorVelocity_mot2_vel_sps(msg_);
  }

private:
  ::roboclaw_ros2::msg::RoboclawMotorVelocity msg_;
};

class Init_RoboclawMotorVelocity_index
{
public:
  Init_RoboclawMotorVelocity_index()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RoboclawMotorVelocity_mot1_vel_sps index(::roboclaw_ros2::msg::RoboclawMotorVelocity::_index_type arg)
  {
    msg_.index = std::move(arg);
    return Init_RoboclawMotorVelocity_mot1_vel_sps(msg_);
  }

private:
  ::roboclaw_ros2::msg::RoboclawMotorVelocity msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::roboclaw_ros2::msg::RoboclawMotorVelocity>()
{
  return roboclaw_ros2::msg::builder::Init_RoboclawMotorVelocity_index();
}

}  // namespace roboclaw_ros2

#endif  // ROBOCLAW_ROS2__MSG__DETAIL__ROBOCLAW_MOTOR_VELOCITY__BUILDER_HPP_
