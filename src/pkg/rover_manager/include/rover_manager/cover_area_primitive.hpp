#ifndef ROVER_MANAGER__COVER_AREA_PRIMITIVE_HPP_
#define ROVER_MANAGER__COVER_AREA_PRIMITIVE_HPP_

#include "rover_manager/primitive_base.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/polygon.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include <utility>
#include <string>

namespace rover_manager
{

/**
 * @brief Primitive for systematic area coverage using a boustrophedon path.
 *
 * Accepts a polygon as a list of vertices and generates a sweep path that
 * covers the area. Sends each waypoint as a Nav2 NavigateToPose goal.
 *
 * Supports RESUME: when cancelled and re-invoked with the SAME polygon,
 * coverage resumes from the waypoint where it was interrupted instead of
 * restarting from the first waypoint.
 *
 * Parameters (yaml):
 *   - swath_width  : distance between adjacent sweep lines (meters)
 *   - sweep_angle  : sweep direction in degrees (0 = aligned with x-axis)
 */
class CoverAreaPrimitive : public PrimitiveBase
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  CoverAreaPrimitive() = default;
  ~CoverAreaPrimitive() override = default;

  void initialize(
    rclcpp::Node::SharedPtr node,
    const std::string & name,
    const std::string & ns) override;

  bool execute(const std::vector<std::string> & args) override;
  void tick() override;
  void cancel() override;

private:
  std::vector<std::pair<double, double>> parse_polygon(
    const std::vector<std::string> & args);
  void generate_boustrophedon_path();
  std::pair<double, double> rotate_point(
    const std::pair<double, double> & p, double angle);
  std::vector<std::pair<double, double>> rotate_points(
    const std::vector<std::pair<double, double>> & pts, double angle);
  std::vector<double> find_polygon_intersections(
    const std::vector<std::pair<double, double>> & pts, double y);
  bool send_waypoint(double x, double y, double yaw);

  // -- Resume support --
  /// Build a stable signature for the current coverage_area_ to detect
  /// whether the new execute() is on the same polygon as the previous one.
  std::string compute_area_signature() const;

  // Nav2 action callbacks
  void goal_response_callback(std::shared_ptr<GoalHandleNav> goal_handle);
  void feedback_callback(
    std::shared_ptr<GoalHandleNav>,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  void result_callback(const GoalHandleNav::WrappedResult & result);

  // ROS2 node
  rclcpp::Node::SharedPtr node_;

  // Nav2 client
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;
  std::shared_ptr<GoalHandleNav> goal_handle_;

  // Coverage state (current invocation)
  std::vector<std::pair<double, double>> coverage_area_;
  std::vector<std::pair<double, double>> coverage_path_;
  size_t current_waypoint_ = 0;
  bool waypoint_sent_ = false;
  bool waypoint_finished_ = false;
  rclcpp_action::ResultCode waypoint_result_;

  // -- Resume state (persists across cancel/execute cycles) --
  size_t saved_waypoint_index_ = 0;
  std::string saved_area_signature_;

  // Polygon publisher for RViz visualization
  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr coverage_pub_;

  // Parameters
  double swath_width_ = 1.5;
  double sweep_angle_ = 0.0;
};

}  // namespace rover_manager

#endif  // ROVER_MANAGER__COVER_AREA_PRIMITIVE_HPP_
