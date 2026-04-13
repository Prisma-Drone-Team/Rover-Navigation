#ifndef ROVER_MANAGER__RETURN_TO_BASE_PRIMITIVE_HPP_
#define ROVER_MANAGER__RETURN_TO_BASE_PRIMITIVE_HPP_

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
 * @brief Primitive to return the rover to its home/base position.
 *
 * The home position is configurable via YAML as either:
 *   - Coordinates: "x,y,yaw" (e.g., "0.0,0.0,0.0")
 *   - TF frame name (e.g., "robot1/map")
 *
 * Internally uses Nav2 NavigateToPose, similar to GotoPrimitive.
 *
 * Parameters (yaml):
 *   - home_pose : home position as "x,y,yaw" or TF frame name
 */
class ReturnToBasePrimitive : public PrimitiveBase
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  ReturnToBasePrimitive() = default;
  ~ReturnToBasePrimitive() override = default;

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

  void goal_response_callback(std::shared_ptr<GoalHandleNav> goal_handle);
  void feedback_callback(
    std::shared_ptr<GoalHandleNav>,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  void result_callback(const GoalHandleNav::WrappedResult & result);

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;
  std::shared_ptr<GoalHandleNav> goal_handle_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  bool goal_finished_ = false;
  rclcpp_action::ResultCode result_code_;

  // Parameters
  std::string home_pose_ = "0.0,0.0,0.0";
};

}  // namespace rover_manager

#endif  // ROVER_MANAGER__RETURN_TO_BASE_PRIMITIVE_HPP_
