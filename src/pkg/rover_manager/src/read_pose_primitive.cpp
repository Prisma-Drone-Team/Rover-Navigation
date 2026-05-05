#include "rover_manager/read_pose_primitive.hpp"

namespace rover_manager
{

void ReadPosePrimitive::initialize(
  rclcpp::Node::SharedPtr node,
  const std::string & name,
  const std::string & ns)
{
  node_     = node;
  name_     = name;
  robot_ns_ = ns;
  logger_   = rclcpp::get_logger(name_);

  try { node_->declare_parameter(name_ + ".lookup_timeout_sec", lookup_timeout_sec_); }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {}
  node_->get_parameter(name_ + ".lookup_timeout_sec", lookup_timeout_sec_);

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/" + robot_ns_ + "/rock_pose", 10);

  RCLCPP_WARN(logger_,
    "[ReadPosePrimitive] initialized: pose_topic='/%s/rock_pose', timeout=%.1fs",
    robot_ns_.c_str(), lookup_timeout_sec_);
}

// ============================================================
// execute
// ============================================================

bool ReadPosePrimitive::execute(const std::vector<std::string> & args)
{
  if (args.empty()) {
    RCLCPP_ERROR(logger_, "[ReadPosePrimitive] no target frame argument");
    status_ = PrimitiveStatus::FAILED;
    return false;
  }

  target_frame_ = args[0];
  instance_predicate_ = name_ + "(" + target_frame_ + ")";
  started_at_ = node_->now();
  status_ = PrimitiveStatus::RUNNING;
  feedback_msg_ = "Reading pose of " + target_frame_;

  RCLCPP_WARN(logger_, "[ReadPosePrimitive] reading pose of '%s'", target_frame_.c_str());
  return true;
}

// ============================================================
// tick — keep retrying the TF lookup until success or timeout
// ============================================================

void ReadPosePrimitive::tick()
{
  if (status_ != PrimitiveStatus::RUNNING) return;

  try {
    auto t = tf_buffer_->lookupTransform(
      "map", target_frame_, tf2::TimePointZero,
      tf2::durationFromSec(0.2));

    geometry_msgs::msg::PoseStamped p;
    p.header = t.header;
    p.pose.position.x = t.transform.translation.x;
    p.pose.position.y = t.transform.translation.y;
    p.pose.position.z = t.transform.translation.z;
    p.pose.orientation = t.transform.rotation;
    pose_pub_->publish(p);

    RCLCPP_WARN(logger_,
      "[ReadPosePrimitive] %s pose in 'map': (%.2f, %.2f, %.2f)",
      target_frame_.c_str(),
      p.pose.position.x, p.pose.position.y, p.pose.position.z);

    feedback_msg_ = "Pose read successfully";
    status_ = PrimitiveStatus::SUCCEEDED;
  } catch (const tf2::TransformException & ex) {
    if ((node_->now() - started_at_).seconds() > lookup_timeout_sec_) {
      RCLCPP_ERROR(logger_,
        "[ReadPosePrimitive] TF lookup of '%s' failed after %.1fs: %s",
        target_frame_.c_str(), lookup_timeout_sec_, ex.what());
      feedback_msg_ = "TF lookup timeout";
      status_ = PrimitiveStatus::FAILED;
    }
    // else: retry on next tick
  }
}

// ============================================================
// cancel
// ============================================================

void ReadPosePrimitive::cancel()
{
  if (status_ != PrimitiveStatus::RUNNING) return;
  status_ = PrimitiveStatus::CANCELLED;
  feedback_msg_ = "Cancelled by manager";
  RCLCPP_WARN(logger_, "[ReadPosePrimitive] Cancelled");
}

}  // namespace rover_manager

// pluginlib export
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rover_manager::ReadPosePrimitive, rover_manager::PrimitiveBase)
