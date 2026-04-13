#include "rover_manager/goto_primitive.hpp"

namespace rover_manager
{

void GotoPrimitive::initialize(
  rclcpp::Node::SharedPtr node,
  const std::string & name,
  const std::string & ns)
{
  node_     = node;
  name_     = name;
  robot_ns_ = ns;
  logger_   = rclcpp::get_logger(name_);

  // Declare parameters with try/catch for already-declared params
  try { node_->declare_parameter(name_ + ".goal_tolerance_xy", goal_tolerance_xy_); }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {}
  try { node_->declare_parameter(name_ + ".goal_tolerance_yaw", goal_tolerance_yaw_); }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {}

  node_->get_parameter(name_ + ".goal_tolerance_xy", goal_tolerance_xy_);
  node_->get_parameter(name_ + ".goal_tolerance_yaw", goal_tolerance_yaw_);

  // Create Nav2 action client with robot namespace
  std::string action_name = "/" + robot_ns_ + "/navigate_to_pose";
  nav2_client_ = rclcpp_action::create_client<NavigateToPose>(node_, action_name);

  // TF2
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_WARN(logger_,
    "[GotoPrimitive] initialized: action='%s', tol_xy=%.2f, tol_yaw=%.2f",
    action_name.c_str(), goal_tolerance_xy_, goal_tolerance_yaw_);
}

// ============================================================
// execute
// ============================================================

bool GotoPrimitive::execute(const std::vector<std::string> & args)
{
  if (args.empty()) {
    RCLCPP_ERROR(logger_, "[GotoPrimitive] execute: no arguments provided");
    status_ = PrimitiveStatus::FAILED;
    return false;
  }

  double x = 0, y = 0, yaw = 0;
  
  // Rejoin args in case the parser split "1.0,2.0,0.0" into separate tokens
  std::string joined;
  for (size_t i = 0; i < args.size(); ++i) {
    if (i > 0) joined += ",";
    joined += args[i];
  }
  instance_predicate_ = name_ + "(" + joined + ")";
  


  if (parse_xyyaw(joined, x, y, yaw)) {
    // Parsed as coordinates
  } else {
    // Try as TF frame name
    try {
      auto tf = tf_buffer_->lookupTransform(
        "map", args[0], tf2::TimePointZero, std::chrono::milliseconds(200));
      x = tf.transform.translation.x;
      y = tf.transform.translation.y;
      tf2::Quaternion quat(
        tf.transform.rotation.x, tf.transform.rotation.y,
        tf.transform.rotation.z, tf.transform.rotation.w);
      double roll, pitch;
      tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(logger_, "[GotoPrimitive] Cannot parse target '%s': %s",
                   args[0].c_str(), ex.what());
      status_ = PrimitiveStatus::FAILED;
      return false;
    }
  }

  return send_goal(x, y, yaw);
}

// ============================================================
// tick
// ============================================================

void GotoPrimitive::tick()
{
  if (status_ != PrimitiveStatus::RUNNING) return;

  // Check if Nav2 goal has finished
  if (goal_finished_) {
    switch (result_code_) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        status_ = PrimitiveStatus::SUCCEEDED;
        feedback_msg_ = "Goal reached";
        RCLCPP_WARN(logger_, "[GotoPrimitive] Goal reached successfully");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        status_ = PrimitiveStatus::FAILED;
        feedback_msg_ = "Goal aborted by Nav2";
        RCLCPP_ERROR(logger_, "[GotoPrimitive] Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        status_ = PrimitiveStatus::CANCELLED;
        feedback_msg_ = "Goal cancelled";
        RCLCPP_WARN(logger_, "[GotoPrimitive] Goal was cancelled");
        break;
      default:
        status_ = PrimitiveStatus::FAILED;
        feedback_msg_ = "Unknown result";
        break;
    }
  }
}

// ============================================================
// cancel
// ============================================================

void GotoPrimitive::cancel()
{
  if (status_ != PrimitiveStatus::RUNNING) return;

  if (nav2_client_ && goal_handle_) {
    nav2_client_->async_cancel_goal(goal_handle_);
  }
  status_ = PrimitiveStatus::CANCELLED;
  feedback_msg_ = "Cancelled by manager";
  goal_finished_ = true;

  RCLCPP_WARN(logger_, "[GotoPrimitive] Cancelling goal");
}

// ============================================================
// send_goal
// ============================================================

bool GotoPrimitive::send_goal(double x, double y, double yaw)
{
  if (!nav2_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(logger_, "[GotoPrimitive] Nav2 action server not available");
    status_ = PrimitiveStatus::FAILED;
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

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&GotoPrimitive::goal_response_callback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(&GotoPrimitive::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&GotoPrimitive::result_callback, this, std::placeholders::_1);

  nav2_client_->async_send_goal(goal_msg, send_goal_options);

  status_ = PrimitiveStatus::RUNNING;
  goal_sent_ = true;
  goal_accepted_ = false;
  goal_finished_ = false;
  feedback_msg_ = "Navigating to (" + std::to_string(x) + ", " + std::to_string(y) + ")";

  RCLCPP_WARN(logger_, "[GotoPrimitive] Sending goal: (%.2f, %.2f, yaw=%.2f)", x, y, yaw);
  return true;
}

// ============================================================
// Nav2 callbacks
// ============================================================

void GotoPrimitive::goal_response_callback(std::shared_ptr<GoalHandleNav> goal_handle)
{
  goal_handle_ = goal_handle;
  if (!goal_handle_) {
    RCLCPP_ERROR(logger_, "[GotoPrimitive] Goal rejected by Nav2");
    goal_accepted_ = false;
    goal_finished_ = true;
    result_code_ = rclcpp_action::ResultCode::ABORTED;
  } else {
    RCLCPP_WARN(logger_, "[GotoPrimitive] Goal accepted by Nav2");
    goal_accepted_ = true;
  }
}

void GotoPrimitive::feedback_callback(
  std::shared_ptr<GoalHandleNav>,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  double dist = feedback->distance_remaining;
  feedback_msg_ = "Distance remaining: " + std::to_string(dist) + " m";
}

void GotoPrimitive::result_callback(const GoalHandleNav::WrappedResult & result)
{
  result_code_ = result.code;
  goal_finished_ = true;
}

// ============================================================
// parse_xyyaw
// ============================================================

bool GotoPrimitive::parse_xyyaw(const std::string & s, double & x, double & y, double & yaw)
{
  std::vector<std::string> toks;
  std::stringstream ss(s);
  std::string tok;
  while (std::getline(ss, tok, ',')) {
    if (!tok.empty()) toks.push_back(tok);
  }

  try {
    if (toks.size() >= 2) {
      x = std::stod(toks[0]);
      y = std::stod(toks[1]);
      yaw = (toks.size() >= 3) ? std::stod(toks[2]) : 0.0;
      return true;
    }
  } catch (...) {
    return false;
  }
  return false;
}

}  // namespace rover_manager

// pluginlib export
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rover_manager::GotoPrimitive, rover_manager::PrimitiveBase)
