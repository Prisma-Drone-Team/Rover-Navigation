#include "rover_manager/take_photo_primitive.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <cstdlib>
#include <ctime>
#include <filesystem>

namespace rover_manager
{

void TakePhotoPrimitive::initialize(
  rclcpp::Node::SharedPtr node,
  const std::string & name,
  const std::string & ns)
{
  node_     = node;
  name_     = name;
  robot_ns_ = ns;
  logger_   = rclcpp::get_logger(name_);

  //std::string default_topic = "/" + robot_ns_ + "/color/image_raw_OFF"; //TEST V4-V5
  std::string default_topic = "/" + robot_ns_ + "/color/image_raw";
  const char * home = std::getenv("HOME");
  std::string default_dir = std::string(home ? home : ".") + "/rock_photos";

  try { node_->declare_parameter(name_ + ".image_topic", default_topic); }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {}
  try { node_->declare_parameter(name_ + ".photo_dir", default_dir); }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {}
  try { node_->declare_parameter(name_ + ".timeout", timeout_); }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {}

  node_->get_parameter(name_ + ".image_topic", image_topic_);
  node_->get_parameter(name_ + ".photo_dir", photo_dir_);
  node_->get_parameter(name_ + ".timeout", timeout_);

  last_image_time_ = node_->now();
  request_time_    = node_->now();

  // Continuous subscription: we keep the most recent frame around so that,
  // when execute() is called, we only have to wait for the next fresh one.
  image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    image_topic_, rclcpp::SensorDataQoS(),
    std::bind(&TakePhotoPrimitive::image_callback, this, std::placeholders::_1));

  RCLCPP_WARN(logger_,
    "[TakePhotoPrimitive] initialized: topic='%s', dir='%s', timeout=%.1fs",
    image_topic_.c_str(), photo_dir_.c_str(), timeout_);
}

void TakePhotoPrimitive::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  last_image_      = msg;
  last_image_time_ = node_->now();
  have_image_      = true;
}

bool TakePhotoPrimitive::execute(const std::vector<std::string> & args)
{
  // Rebuild the predicate the same way GotoPrimitive does, so the manager
  // publishes succeeded(take_photo(<label>)).
  std::string joined;
  for (size_t i = 0; i < args.size(); ++i) {
    if (i > 0) joined += ",";
    joined += args[i];
  }
  instance_predicate_ = name_ + "(" + joined + ")";

  label_        = args.empty() ? "photo" : args[0];
  request_time_ = node_->now();
  status_       = PrimitiveStatus::RUNNING;
  feedback_msg_ = "Waiting for a fresh frame";

  RCLCPP_WARN(logger_, "[TakePhotoPrimitive] capturing '%s'", label_.c_str());
  return true;
}

void TakePhotoPrimitive::tick()
{
  if (status_ != PrimitiveStatus::RUNNING) return;

  // Timeout guard: never hang the cognitive sequence.
  if ((node_->now() - request_time_).seconds() > timeout_) {
    status_       = PrimitiveStatus::FAILED;
    feedback_msg_ = "No frame received within timeout";
    RCLCPP_ERROR(logger_, "[TakePhotoPrimitive] timeout, no frame on '%s'",
                 image_topic_.c_str());
    return;
  }

  // Only accept a frame that arrived AFTER execute() (fresh viewpoint).
  if (!have_image_ || last_image_time_ <= request_time_) return;

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(last_image_, "bgr8");
  } catch (const cv_bridge::Exception & e) {
    status_       = PrimitiveStatus::FAILED;
    feedback_msg_ = std::string("cv_bridge error: ") + e.what();
    RCLCPP_ERROR(logger_, "[TakePhotoPrimitive] cv_bridge: %s", e.what());
    return;
  }

  std::error_code ec;
  std::filesystem::create_directories(photo_dir_, ec);

  std::time_t t = std::time(nullptr);
  char stamp[32];
  std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&t));

  std::string path = photo_dir_ + "/rockg_" + label_ + "_" + stamp + ".png";

  if (cv::imwrite(path, cv_ptr->image)) {
    status_       = PrimitiveStatus::SUCCEEDED;
    feedback_msg_ = "Saved " + path;
    RCLCPP_WARN(logger_, "[TakePhotoPrimitive] saved %s", path.c_str());
  } else {
    status_       = PrimitiveStatus::FAILED;
    feedback_msg_ = "imwrite failed for " + path;
    RCLCPP_ERROR(logger_, "[TakePhotoPrimitive] imwrite failed: %s", path.c_str());
  }
}

void TakePhotoPrimitive::cancel()
{
  if (status_ != PrimitiveStatus::RUNNING) return;
  status_       = PrimitiveStatus::CANCELLED;
  feedback_msg_ = "Cancelled by manager";
  RCLCPP_WARN(logger_, "[TakePhotoPrimitive] cancelled");
}

}  // namespace rover_manager

// pluginlib export
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rover_manager::TakePhotoPrimitive, rover_manager::PrimitiveBase)
