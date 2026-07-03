#include "rover_manager/cover_area_primitive.hpp"
#include <regex>
#include <cmath>
#include <algorithm>

namespace rover_manager
{

void CoverAreaPrimitive::initialize(
  rclcpp::Node::SharedPtr node,
  const std::string & name,
  const std::string & ns)
{
  node_     = node;
  name_     = name;
  robot_ns_ = ns;
  logger_   = rclcpp::get_logger(name_);

  try { node_->declare_parameter(name_ + ".swath_width", swath_width_); }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {}
  try { node_->declare_parameter(name_ + ".sweep_angle", sweep_angle_); }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {}

  node_->get_parameter(name_ + ".swath_width", swath_width_);
  node_->get_parameter(name_ + ".sweep_angle", sweep_angle_);

  std::string action_name = "/" + robot_ns_ + "/navigate_to_pose";
  nav2_client_ = rclcpp_action::create_client<NavigateToPose>(node_, action_name);

  coverage_pub_ = node_->create_publisher<geometry_msgs::msg::Polygon>(
    "/" + robot_ns_ + "/coverage_area", 10);

  RCLCPP_WARN(logger_,
    "[CoverAreaPrimitive] initialized: swath=%.2f m, angle=%.1f deg, action='%s'",
    swath_width_, sweep_angle_, action_name.c_str());
}

// ============================================================
// execute
// ============================================================

bool CoverAreaPrimitive::execute(const std::vector<std::string> & args)
{
  
  std::string joined;
for (size_t i = 0; i < args.size(); ++i) {
  if (i > 0) joined += ",";
  joined += args[i];
}
instance_predicate_ = name_ + "(" + joined + ")";

  // Parse polygon vertices from arguments
  coverage_area_ = parse_polygon(args);

  if (coverage_area_.size() < 3) {
    RCLCPP_ERROR(logger_,
      "[CoverAreaPrimitive] Need at least 3 vertices, got %zu", coverage_area_.size());
    status_ = PrimitiveStatus::FAILED;
    return false;
  }

  // Publish polygon for visualization
  auto poly_msg = geometry_msgs::msg::Polygon();
  for (const auto & pt : coverage_area_) {
    geometry_msgs::msg::Point32 p;
    p.x = pt.first;
    p.y = pt.second;
    p.z = 0.0;
    poly_msg.points.push_back(p);
  }
  coverage_pub_->publish(poly_msg);

  // Generate boustrophedon path
  generate_boustrophedon_path();

  if (coverage_path_.empty()) {
    RCLCPP_ERROR(logger_, "[CoverAreaPrimitive] Generated empty path");
    status_ = PrimitiveStatus::FAILED;
    return false;
  }

  current_waypoint_ = 0;
  waypoint_sent_ = false;
  waypoint_finished_ = false;
  status_ = PrimitiveStatus::RUNNING;
  feedback_msg_ = "Starting coverage with " + std::to_string(coverage_path_.size()) + " waypoints";

  RCLCPP_WARN(logger_,
    "[CoverAreaPrimitive] Starting coverage: %zu vertices, %zu waypoints, swath=%.2f m",
    coverage_area_.size(), coverage_path_.size(), swath_width_);

  return true;
}

// ============================================================
// tick — anytime execution: one waypoint at a time
// ============================================================

void CoverAreaPrimitive::tick()
{
  if (status_ != PrimitiveStatus::RUNNING) return;

  // Check if current waypoint finished
  if (waypoint_sent_ && waypoint_finished_) {
    if (waypoint_result_ == rclcpp_action::ResultCode::SUCCEEDED) {
      // Move to next waypoint
      current_waypoint_++;
      waypoint_sent_ = false;
      waypoint_finished_ = false;

      RCLCPP_WARN(logger_, "[CoverAreaPrimitive] Waypoint %zu/%zu reached",
                  current_waypoint_, coverage_path_.size());

      // Check if coverage is complete
      if (current_waypoint_ >= coverage_path_.size()) {
        status_ = PrimitiveStatus::SUCCEEDED;
        feedback_msg_ = "Coverage completed";
        RCLCPP_WARN(logger_, "[CoverAreaPrimitive] Coverage completed!");

        // Publish empty polygon to clear visualization
        coverage_pub_->publish(geometry_msgs::msg::Polygon());
        return;
      }
    } else {
      // Waypoint failed — skip to next 
      RCLCPP_WARN(logger_, "[CoverAreaPrimitive] Waypoint %zu failed, skipping to next",
                  current_waypoint_);
      current_waypoint_++;
      waypoint_sent_ = false;
      waypoint_finished_ = false;

      if (current_waypoint_ >= coverage_path_.size()) {
        status_ = PrimitiveStatus::SUCCEEDED;
        feedback_msg_ = "Coverage completed (some waypoints skipped)";
        RCLCPP_WARN(logger_, "[CoverAreaPrimitive] Coverage completed with skipped waypoints");
        coverage_pub_->publish(geometry_msgs::msg::Polygon());
        return;
      }
    }
  }

  // Send next waypoint if not already sent
  if (!waypoint_sent_ && current_waypoint_ < coverage_path_.size()) {
    auto wp = coverage_path_[current_waypoint_];
    feedback_msg_ = "Waypoint " + std::to_string(current_waypoint_ + 1) + "/" +
                    std::to_string(coverage_path_.size());

    if (send_waypoint(wp.first, wp.second, 0.0)) {
      waypoint_sent_ = true;
      waypoint_finished_ = false;
    } else {
      status_ = PrimitiveStatus::FAILED;
      feedback_msg_ = "Failed to send waypoint";
    }
  }
}

// ============================================================
// cancel
// ============================================================

void CoverAreaPrimitive::cancel()
{
  if (status_ != PrimitiveStatus::RUNNING) return;

  if (nav2_client_ && goal_handle_) {
    nav2_client_->async_cancel_goal(goal_handle_);
  }
  status_ = PrimitiveStatus::CANCELLED;
  feedback_msg_ = "Coverage cancelled at waypoint " + std::to_string(current_waypoint_);
  RCLCPP_WARN(logger_, "[CoverAreaPrimitive] Cancelled at waypoint %zu/%zu",
              current_waypoint_, coverage_path_.size());

  // Clear visualization
  coverage_pub_->publish(geometry_msgs::msg::Polygon());
}

// ============================================================
// parse_polygon — extract vertices from arguments
// ============================================================

std::vector<std::pair<double, double>> CoverAreaPrimitive::parse_polygon(
  const std::vector<std::string> & args)
{
  std::vector<std::pair<double, double>> points;

  // Rejoin all args to handle both formats:
  //   cover_area((1,2)(3,4)(5,6))  -> args might be split
  //   cover_area(1,2,3,4,5,6)      -> flat list of coordinates
  std::string joined;
  for (size_t i = 0; i < args.size(); ++i) {
    if (i > 0) joined += ",";
    joined += args[i];
  }

  // Try format with parentheses: (x1,y1)(x2,y2)...
  std::regex point_regex(R"(\(([^,]+),([^)]+)\))");
  std::sregex_iterator iter(joined.begin(), joined.end(), point_regex);
  std::sregex_iterator end;

  while (iter != end) {
    try {
      double x = std::stod((*iter)[1]);
      double y = std::stod((*iter)[2]);
      points.push_back({x, y});
    } catch (...) {}
    ++iter;
  }

  // If no parenthesized points found, try flat list: x1,y1,x2,y2,...
  if (points.empty()) {
    std::vector<double> coords;
    std::stringstream ss(joined);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
      try { coords.push_back(std::stod(tok)); }
      catch (...) {}
    }
    for (size_t i = 0; i + 1 < coords.size(); i += 2) {
      points.push_back({coords[i], coords[i + 1]});
    }
  }

  for (const auto & pt : points) {
    RCLCPP_WARN(logger_, "[CoverAreaPrimitive] Parsed vertex: (%.2f, %.2f)", pt.first, pt.second);
  }

  return points;
}

// ============================================================
// Boustrophedon path generation
// ============================================================

void CoverAreaPrimitive::generate_boustrophedon_path()
{
  if (coverage_area_.size() < 3) return;

  double angle_rad = sweep_angle_ * M_PI / 180.0;
  auto rotated_pts = rotate_points(coverage_area_, -angle_rad);

  // Bounding box
  double min_x = rotated_pts[0].first, max_x = rotated_pts[0].first;
  double min_y = rotated_pts[0].second, max_y = rotated_pts[0].second;

  for (const auto & p : rotated_pts) {
    min_x = std::min(min_x, p.first);
    max_x = std::max(max_x, p.first);
    min_y = std::min(min_y, p.second);
    max_y = std::max(max_y, p.second);
  }

  coverage_path_.clear();
  int line_count = 0;

  for (double y = min_y + swath_width_ / 2; y <= max_y; y += swath_width_) {
    auto intersections = find_polygon_intersections(rotated_pts, y);
    if (intersections.size() >= 2) {
      std::sort(intersections.begin(), intersections.end());
      double x_start = intersections.front();
      double x_end = intersections.back();

      if (line_count % 2 == 0) {
        coverage_path_.push_back({x_start, y});
        coverage_path_.push_back({x_end, y});
      } else {
        coverage_path_.push_back({x_end, y});
        coverage_path_.push_back({x_start, y});
      }
      line_count++;
    }
  }

  // Rotate waypoints back to original orientation
  for (auto & p : coverage_path_) {
    p = rotate_point(p, angle_rad);
  }

  RCLCPP_WARN(logger_, "[CoverAreaPrimitive] Generated %zu waypoints", coverage_path_.size());
}

std::pair<double, double> CoverAreaPrimitive::rotate_point(
  const std::pair<double, double> & p, double angle)
{
  double cos_a = std::cos(angle), sin_a = std::sin(angle);
  return {p.first * cos_a - p.second * sin_a,
          p.first * sin_a + p.second * cos_a};
}

std::vector<std::pair<double, double>> CoverAreaPrimitive::rotate_points(
  const std::vector<std::pair<double, double>> & pts, double angle)
{
  std::vector<std::pair<double, double>> rotated;
  for (const auto & p : pts) {
    rotated.push_back(rotate_point(p, angle));
  }
  return rotated;
}

std::vector<double> CoverAreaPrimitive::find_polygon_intersections(
  const std::vector<std::pair<double, double>> & pts, double y)
{
  std::vector<double> intersections;
  size_t n = pts.size();
  for (size_t i = 0; i < n; i++) {
    auto p1 = pts[i];
    auto p2 = pts[(i + 1) % n];
    if ((p1.second <= y && y <= p2.second) || (p2.second <= y && y <= p1.second)) {
      if (p1.second != p2.second) {
        double t = (y - p1.second) / (p2.second - p1.second);
        double x = p1.first + t * (p2.first - p1.first);
        intersections.push_back(x);
      }
    }
  }
  return intersections;
}

// ============================================================
// send_waypoint
// ============================================================

bool CoverAreaPrimitive::send_waypoint(double x, double y, double yaw)
{
  if (!nav2_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(logger_, "[CoverAreaPrimitive] Nav2 action server not available");
    return false;
  }

  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.header.stamp = node_->now();
  goal_msg.pose.pose.position.x = x;
  goal_msg.pose.pose.position.y = y;
  goal_msg.pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  goal_msg.pose.pose.orientation.x = q.x();
  goal_msg.pose.pose.orientation.y = q.y();
  goal_msg.pose.pose.orientation.z = q.z();
  goal_msg.pose.pose.orientation.w = q.w();

  auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  opts.goal_response_callback =
    std::bind(&CoverAreaPrimitive::goal_response_callback, this, std::placeholders::_1);
  opts.feedback_callback =
    std::bind(&CoverAreaPrimitive::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
  opts.result_callback =
    std::bind(&CoverAreaPrimitive::result_callback, this, std::placeholders::_1);

  nav2_client_->async_send_goal(goal_msg, opts);

  RCLCPP_WARN(logger_, "[CoverAreaPrimitive] Sending waypoint %zu: (%.2f, %.2f)",
              current_waypoint_, x, y);
  return true;
}

// ============================================================
// Nav2 callbacks
// ============================================================

void CoverAreaPrimitive::goal_response_callback(std::shared_ptr<GoalHandleNav> goal_handle)
{
  goal_handle_ = goal_handle;
  if (!goal_handle_) {
    RCLCPP_ERROR(logger_, "[CoverAreaPrimitive] Waypoint rejected by Nav2");
    waypoint_finished_ = true;
    waypoint_result_ = rclcpp_action::ResultCode::ABORTED;
  }
}

void CoverAreaPrimitive::feedback_callback(
  std::shared_ptr<GoalHandleNav>,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  feedback_msg_ = "Waypoint " + std::to_string(current_waypoint_ + 1) + "/" +
                  std::to_string(coverage_path_.size()) +
                  " - dist: " + std::to_string(feedback->distance_remaining) + " m";
}

void CoverAreaPrimitive::result_callback(const GoalHandleNav::WrappedResult & result)
{
  waypoint_result_ = result.code;
  waypoint_finished_ = true;
}

}  // namespace rover_manager

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rover_manager::CoverAreaPrimitive, rover_manager::PrimitiveBase)
