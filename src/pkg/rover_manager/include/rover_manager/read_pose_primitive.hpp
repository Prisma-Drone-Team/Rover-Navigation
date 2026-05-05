#ifndef ROVER_MANAGER__READ_POSE_PRIMITIVE_HPP_
#define ROVER_MANAGER__READ_POSE_PRIMITIVE_HPP_

#include "rover_manager/primitive_base.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace rover_manager
{

/**
 * @brief Primitive that reads a TF frame's pose in 'map' and publishes it.
 *
 * Used by the seed during rock_response: leo1 (or leo2) calls
 * read_pose(rockg) to capture and broadcast the rock pose.
 *
 * The pose is published on '<robot_ns>/rock_pose' (PoseStamped, frame 'map').
 *
 * Parameters (yaml):
 *   - lookup_timeout_sec : how long to keep retrying TF lookup before failing
 *                         (default 3.0 seconds)
 */
class ReadPosePrimitive : public PrimitiveBase
{
public:
  ReadPosePrimitive() = default;
  ~ReadPosePrimitive() override = default;

  void initialize(
    rclcpp::Node::SharedPtr node,
    const std::string & name,
    const std::string & ns) override;

  bool execute(const std::vector<std::string> & args) override;
  void tick() override;
  void cancel() override;

private:
  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  std::string target_frame_;
  rclcpp::Time started_at_;
  double lookup_timeout_sec_ = 3.0;
};

}  // namespace rover_manager

#endif  // ROVER_MANAGER__READ_POSE_PRIMITIVE_HPP_
