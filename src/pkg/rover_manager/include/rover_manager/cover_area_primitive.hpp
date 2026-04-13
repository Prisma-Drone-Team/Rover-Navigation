#ifndef ROVER_MANAGER__COVER_AREA_PRIMITIVE_HPP_
#define ROVER_MANAGER__COVER_AREA_PRIMITIVE_HPP_

#include "rover_manager/primitive_base.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/polygon.hpp"

#include <tf2/LinearMath/Quaternion.h>

namespace rover_manager
{

/**
 * @brief Primitive for area coverage using boustrophedon decomposition.
 *
 * Receives a polygon (list of vertices), generates a boustrophedon
 * (lawn-mower) path, and executes it waypoint by waypoint via Nav2.
 *
 * This primitive is "anytime": it tracks the current waypoint index,
 * can be cancelled at any point, and reports progress via feedback.
 *
 * Command format: cover_area((x1,y1)(x2,y2)(x3,y3)...)
 *
 * Parameters (yaml):
 *   - swath_width     : distance between parallel passes (meters)
 *   - sweep_angle     : angle of the sweep pattern (degrees, 0 = aligned with X)
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

  void reset() override
  {
    PrimitiveBase::reset();
    coverage_path_.clear();
    coverage_area_.clear();
    current_waypoint_ = 0;
    waypoint_sent_ = false;
    waypoint_finished_ = false;
  }

private:
  // Polygon parsing
  std::vector<std::pair<double, double>> parse_polygon(const std::vector<std::string> & args);

  // Boustrophedon path generation
  void generate_boustrophedon_path();
  std::pair<double, double> rotate_point(const std::pair<double, double> & p, double angle);
  std::vector<std::pair<double, double>> rotate_points(
    const std::vector<std::pair<double, double>> & pts, double angle);
  std::vector<double> find_polygon_intersections(
    const std::vector<std::pair<double, double>> & pts, double y);

  // Send a single waypoint to Nav2
  bool send_waypoint(double x, double y, double yaw);

  // Nav2 callbacks
  void goal_response_callback(std::shared_ptr<GoalHandleNav> goal_handle);
  void feedback_callback(
    std::shared_ptr<GoalHandleNav>,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  void result_callback(const GoalHandleNav::WrappedResult & result);

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;
  std::shared_ptr<GoalHandleNav> goal_handle_;

  // Coverage polygon publisher (for visualization)
  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr coverage_pub_;

  // Coverage state
  std::vector<std::pair<double, double>> coverage_area_;
  std::vector<std::pair<double, double>> coverage_path_;
  size_t current_waypoint_ = 0;
  bool waypoint_sent_ = false;
  bool waypoint_finished_ = false;
  rclcpp_action::ResultCode waypoint_result_;

  // Parameters
  double swath_width_ = 1.0;
  double sweep_angle_ = 0.0;  // degrees
};

}  // namespace rover_manager

#endif  // ROVER_MANAGER__COVER_AREA_PRIMITIVE_HPP_
