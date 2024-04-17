// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from roboclaw_ros2:msg/RoboclawMotorVelocity.idl
// generated code does not contain a copyright notice

#ifndef ROBOCLAW_ROS2__MSG__DETAIL__ROBOCLAW_MOTOR_VELOCITY__STRUCT_HPP_
#define ROBOCLAW_ROS2__MSG__DETAIL__ROBOCLAW_MOTOR_VELOCITY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__roboclaw_ros2__msg__RoboclawMotorVelocity __attribute__((deprecated))
#else
# define DEPRECATED__roboclaw_ros2__msg__RoboclawMotorVelocity __declspec(deprecated)
#endif

namespace roboclaw_ros2
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RoboclawMotorVelocity_
{
  using Type = RoboclawMotorVelocity_<ContainerAllocator>;

  explicit RoboclawMotorVelocity_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->index = 0l;
      this->mot1_vel_sps = 0l;
      this->mot2_vel_sps = 0l;
    }
  }

  explicit RoboclawMotorVelocity_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->index = 0l;
      this->mot1_vel_sps = 0l;
      this->mot2_vel_sps = 0l;
    }
  }

  // field types and members
  using _index_type =
    int32_t;
  _index_type index;
  using _mot1_vel_sps_type =
    int32_t;
  _mot1_vel_sps_type mot1_vel_sps;
  using _mot2_vel_sps_type =
    int32_t;
  _mot2_vel_sps_type mot2_vel_sps;

  // setters for named parameter idiom
  Type & set__index(
    const int32_t & _arg)
  {
    this->index = _arg;
    return *this;
  }
  Type & set__mot1_vel_sps(
    const int32_t & _arg)
  {
    this->mot1_vel_sps = _arg;
    return *this;
  }
  Type & set__mot2_vel_sps(
    const int32_t & _arg)
  {
    this->mot2_vel_sps = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    roboclaw_ros2::msg::RoboclawMotorVelocity_<ContainerAllocator> *;
  using ConstRawPtr =
    const roboclaw_ros2::msg::RoboclawMotorVelocity_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<roboclaw_ros2::msg::RoboclawMotorVelocity_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<roboclaw_ros2::msg::RoboclawMotorVelocity_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      roboclaw_ros2::msg::RoboclawMotorVelocity_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<roboclaw_ros2::msg::RoboclawMotorVelocity_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      roboclaw_ros2::msg::RoboclawMotorVelocity_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<roboclaw_ros2::msg::RoboclawMotorVelocity_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<roboclaw_ros2::msg::RoboclawMotorVelocity_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<roboclaw_ros2::msg::RoboclawMotorVelocity_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__roboclaw_ros2__msg__RoboclawMotorVelocity
    std::shared_ptr<roboclaw_ros2::msg::RoboclawMotorVelocity_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__roboclaw_ros2__msg__RoboclawMotorVelocity
    std::shared_ptr<roboclaw_ros2::msg::RoboclawMotorVelocity_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RoboclawMotorVelocity_ & other) const
  {
    if (this->index != other.index) {
      return false;
    }
    if (this->mot1_vel_sps != other.mot1_vel_sps) {
      return false;
    }
    if (this->mot2_vel_sps != other.mot2_vel_sps) {
      return false;
    }
    return true;
  }
  bool operator!=(const RoboclawMotorVelocity_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RoboclawMotorVelocity_

// alias to use template instance with default allocator
using RoboclawMotorVelocity =
  roboclaw_ros2::msg::RoboclawMotorVelocity_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace roboclaw_ros2

#endif  // ROBOCLAW_ROS2__MSG__DETAIL__ROBOCLAW_MOTOR_VELOCITY__STRUCT_HPP_
