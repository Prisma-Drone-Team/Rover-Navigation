#ifndef ROVER_MANAGER__GOTO_PRIMITIVE_HPP_
#define ROVER_MANAGER__GOTO_PRIMITIVE_HPP_

#include "rover_manager/primitive_base.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace rover_manager
{

/**
 * @brief Primitive for point-to-point navigation via Nav2.
 *
 * Accepts a target as x,y,yaw coordinates or as a TF frame name.
 * Wraps the Nav2 NavigateToPose action client.
 *
 * Parameters (yaml):
 *   - goal_tolerance_xy   : xy tolerance for goal reaching (informational)
 *   - goal_tolerance_yaw  : yaw tolerance for goal reaching (informational)
 */
class GotoPrimitive : public PrimitiveBase
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  GotoPrimitive() = default;
  ~GotoPrimitive() override = default;

  void initialize(
    rclcpp::Node::SharedPtr node,
    const std::string & name,
    const std::string & ns) override;

  bool execute(const std::vector<std::string> & args) override;
  void tick() override;
  void cancel() override;

private:
  bool parse_xyyaw(const std::string & s, double & x, double & y, double & yaw);
  bool send_goal(double x, double y, double yaw);

  // Nav2 action client callbacks
  void goal_response_callback(std::shared_ptr<GoalHandleNav> goal_handle);
  void feedback_callback(
    std::shared_ptr<GoalHandleNav>,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  void result_callback(const GoalHandleNav::WrappedResult & result);

  // ROS2 node (weak reference)
  rclcpp::Node::SharedPtr node_;

  // Nav2 action client
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;
  std::shared_ptr<GoalHandleNav> goal_handle_;

  // TF2 for frame-based goals
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // State
  bool goal_sent_ = false;
  bool goal_accepted_ = false;
  bool goal_finished_ = false;
  rclcpp_action::ResultCode result_code_;

  // Parameters
  double goal_tolerance_xy_  = 0.25;
  double goal_tolerance_yaw_ = 0.25;
};

}  // namespace rover_manager

#endif  // ROVER_MANAGER__GOTO_PRIMITIVE_HPP_
