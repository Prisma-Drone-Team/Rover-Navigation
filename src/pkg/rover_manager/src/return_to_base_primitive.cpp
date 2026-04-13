#include "rover_manager/return_to_base_primitive.hpp"

namespace rover_manager
{

void ReturnToBasePrimitive::initialize(
  rclcpp::Node::SharedPtr node,
  const std::string & name,
  const std::string & ns)
{
  node_     = node;
  name_     = name;
  robot_ns_ = ns;
  logger_   = rclcpp::get_logger(name_);

  try { node_->declare_parameter(name_ + ".home_pose", home_pose_); }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {}

  node_->get_parameter(name_ + ".home_pose", home_pose_);

  std::string action_name = "/" + robot_ns_ + "/navigate_to_pose";
  nav2_client_ = rclcpp_action::create_client<NavigateToPose>(node_, action_name);

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_WARN(logger_,
    "[ReturnToBasePrimitive] initialized: home_pose='%s', action='%s'",
    home_pose_.c_str(), action_name.c_str());
}

bool ReturnToBasePrimitive::execute(const std::vector<std::string> & /*args*/)
{
  // Ignores arguments — always goes to home_pose_
  double x = 0, y = 0, yaw = 0;

 instance_predicate_ = "return_to_base";

  if (parse_xyyaw(home_pose_, x, y, yaw)) {
    return send_goal(x, y, yaw);
  }

  // Try as TF frame
  try {
    auto tf = tf_buffer_->lookupTransform(
      "map", home_pose_, tf2::TimePointZero, std::chrono::milliseconds(200));
    x = tf.transform.translation.x;
    y = tf.transform.translation.y;
    tf2::Quaternion quat(
      tf.transform.rotation.x, tf.transform.rotation.y,
      tf.transform.rotation.z, tf.transform.rotation.w);
    double roll, pitch;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return send_goal(x, y, yaw);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "[ReturnToBasePrimitive] Cannot resolve home_pose '%s': %s",
                 home_pose_.c_str(), ex.what());
    status_ = PrimitiveStatus::FAILED;
    return false;
  }
}

void ReturnToBasePrimitive::tick()
{
  if (status_ != PrimitiveStatus::RUNNING) return;

  if (goal_finished_) {
    switch (result_code_) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        status_ = PrimitiveStatus::SUCCEEDED;
        feedback_msg_ = "Returned to base";
        RCLCPP_WARN(logger_, "[ReturnToBasePrimitive] Returned to base successfully");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        status_ = PrimitiveStatus::FAILED;
        feedback_msg_ = "Return aborted by Nav2";
        RCLCPP_ERROR(logger_, "[ReturnToBasePrimitive] Return was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        status_ = PrimitiveStatus::CANCELLED;
        feedback_msg_ = "Return cancelled";
        break;
      default:
        status_ = PrimitiveStatus::FAILED;
        feedback_msg_ = "Unknown result";
        break;
    }
  }
}

void ReturnToBasePrimitive::cancel()
{
  if (status_ != PrimitiveStatus::RUNNING) return;

  if (nav2_client_ && goal_handle_) {
    nav2_client_->async_cancel_goal(goal_handle_);
  }
  status_ = PrimitiveStatus::CANCELLED;
  feedback_msg_ = "Cancelled";
  goal_finished_ = true;
  RCLCPP_WARN(logger_, "[ReturnToBasePrimitive] Cancelling return");
}

bool ReturnToBasePrimitive::send_goal(double x, double y, double yaw)
{
  if (!nav2_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(logger_, "[ReturnToBasePrimitive] Nav2 action server not available");
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

  auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  opts.goal_response_callback =
    std::bind(&ReturnToBasePrimitive::goal_response_callback, this, std::placeholders::_1);
  opts.feedback_callback =
    std::bind(&ReturnToBasePrimitive::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
  opts.result_callback =
    std::bind(&ReturnToBasePrimitive::result_callback, this, std::placeholders::_1);

  nav2_client_->async_send_goal(goal_msg, opts);

  status_ = PrimitiveStatus::RUNNING;
  goal_finished_ = false;
  feedback_msg_ = "Returning to base (" + std::to_string(x) + ", " + std::to_string(y) + ")";

  RCLCPP_WARN(logger_, "[ReturnToBasePrimitive] Returning to (%.2f, %.2f, yaw=%.2f)", x, y, yaw);
  return true;
}

void ReturnToBasePrimitive::goal_response_callback(std::shared_ptr<GoalHandleNav> goal_handle)
{
  goal_handle_ = goal_handle;
  if (!goal_handle_) {
    RCLCPP_ERROR(logger_, "[ReturnToBasePrimitive] Goal rejected");
    goal_finished_ = true;
    result_code_ = rclcpp_action::ResultCode::ABORTED;
  }
}

void ReturnToBasePrimitive::feedback_callback(
  std::shared_ptr<GoalHandleNav>,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  feedback_msg_ = "Returning - distance: " + std::to_string(feedback->distance_remaining) + " m";
}

void ReturnToBasePrimitive::result_callback(const GoalHandleNav::WrappedResult & result)
{
  result_code_ = result.code;
  goal_finished_ = true;
}

bool ReturnToBasePrimitive::parse_xyyaw(const std::string & s, double & x, double & y, double & yaw)
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
  } catch (...) { return false; }
  return false;
}

}  // namespace rover_manager

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rover_manager::ReturnToBasePrimitive, rover_manager::PrimitiveBase)
