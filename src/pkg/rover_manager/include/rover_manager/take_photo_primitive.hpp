#ifndef ROVER_MANAGER__TAKE_PHOTO_PRIMITIVE_HPP_
#define ROVER_MANAGER__TAKE_PHOTO_PRIMITIVE_HPP_

#include "rover_manager/primitive_base.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <string>
#include <vector>

namespace rover_manager
{

/**
 * @brief Primitive that captures a single camera frame and saves it to disk.
 *
 * Subscribes to the robot's color image topic. On execute() it waits for the
 * first frame received AFTER the call, so the photo reflects the CURRENT
 * viewpoint and not a buffered stale frame; it then converts the frame with
 * cv_bridge and writes a PNG.
 *
 * The instance predicate is take_photo(<label>), so the LTM can gate the next
 * schema on succeeded(take_photo(<label>)) exactly like it does for goto.
 *
 * Parameters (yaml, namespace = primitive name "take_photo"):
 *   - image_topic : color image topic    (default "/<ns>/color/image_raw")
 *   - photo_dir   : output directory      (default "$HOME/rock_photos")
 *   - timeout     : max wait for a frame  (default 5.0 s)
 */
class TakePhotoPrimitive : public PrimitiveBase
{
public:
  TakePhotoPrimitive() = default;
  ~TakePhotoPrimitive() override = default;

  void initialize(
    rclcpp::Node::SharedPtr node,
    const std::string & name,
    const std::string & ns) override;

  bool execute(const std::vector<std::string> & args) override;
  void tick() override;
  void cancel() override;

private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  // Latest frame received from the camera (continuously updated)
  sensor_msgs::msg::Image::ConstSharedPtr last_image_;
  rclcpp::Time last_image_time_;
  bool have_image_ = false;

  // Per-execution state
  rclcpp::Time request_time_;
  std::string label_;

  // Parameters
  std::string image_topic_;
  std::string photo_dir_;
  double timeout_ = 5.0;
};

}  // namespace rover_manager

#endif  // ROVER_MANAGER__TAKE_PHOTO_PRIMITIVE_HPP_
