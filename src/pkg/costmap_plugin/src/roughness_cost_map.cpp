#include "costmap_plugin/roughness_cost_map.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include <cmath>
#include <algorithm>

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;

namespace costmap_plugin
{

void RoughnessCostMap::initialize(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::string & name)
{
  name_   = name;
  logger_ = rclcpp::get_logger(name_);

  node->declare_parameter(name_ + ".roughness_threshold", roughness_threshold_);
  node->declare_parameter(name_ + ".cost_scaling_factor", cost_scaling_factor_);
  node->declare_parameter(name_ + ".kernel_size",         kernel_size_);
  node->declare_parameter(name_ + ".weight",              weight_);

  node->get_parameter(name_ + ".roughness_threshold", roughness_threshold_);
  node->get_parameter(name_ + ".cost_scaling_factor", cost_scaling_factor_);
  node->get_parameter(name_ + ".kernel_size",         kernel_size_);
  node->get_parameter(name_ + ".weight",              weight_);

  // Ensure kernel_size is odd and >= 3
  if (kernel_size_ < 3) kernel_size_ = 3;
  if (kernel_size_ % 2 == 0) kernel_size_ += 1;

  RCLCPP_WARN(
    logger_,
    "[RoughnessCostMap] initialized: threshold=%.3f m, scaling=%.1f, kernel=%dx%d, weight=%.2f",
    roughness_threshold_, cost_scaling_factor_, kernel_size_, kernel_size_, weight_);
}

// ============================================================
// setElevationData (from GradientLayer)
// ============================================================

void RoughnessCostMap::setElevationData(
  const std::vector<float> & elevation_map,
  unsigned int size_x,
  unsigned int size_y,
  double resolution)
{
  elevation_map_ = elevation_map;
  size_x_ = size_x;
  size_y_ = size_y;
  resolution_ = resolution;
  has_elevation_data_ = true;
}

// ============================================================
// computeRoughness
// ============================================================

double RoughnessCostMap::computeRoughness(
  unsigned int mx,
  unsigned int my) const
{
  int half = kernel_size_ / 2;
  double sum = 0.0;
  double sum_sq = 0.0;
  int count = 0;

  for (int dy = -half; dy <= half; ++dy) {
    for (int dx = -half; dx <= half; ++dx) {
      int nx = static_cast<int>(mx) + dx;
      int ny = static_cast<int>(my) + dy;

      if (nx < 0 || ny < 0 ||
          nx >= static_cast<int>(size_x_) ||
          ny >= static_cast<int>(size_y_))
      {
        continue;
      }

      unsigned int idx = ny * size_x_ + nx;
      if (idx >= elevation_map_.size()) continue;

      float h = elevation_map_[idx];
      if (h <= INVALID_ELEVATION) continue;

      sum += h;
      sum_sq += h * h;
      ++count;
    }
  }

  // Need at least half the kernel cells to have valid data
  int min_valid = (kernel_size_ * kernel_size_) / 2;
  if (count < min_valid) return 0.0;

  double mean = sum / count;
  double variance = (sum_sq / count) - (mean * mean);

  // Guard against floating point errors
  if (variance < 0.0) return 0.0;

  return std::sqrt(variance);
}

// ============================================================
// computeCost
// ============================================================

unsigned char RoughnessCostMap::computeCost(
  unsigned int mx,
  unsigned int my,
  const nav2_costmap_2d::Costmap2D & /*master_grid*/,
  std::shared_ptr<octomap::ColorOcTree> /*octree*/)
{
  if (!has_elevation_data_) return FREE_SPACE;

  double roughness = computeRoughness(mx, my);

  if (roughness >= roughness_threshold_) {
    return LETHAL_OBSTACLE;
  } else if (roughness > 0.001) {
    double cost = cost_scaling_factor_ * roughness;
    return static_cast<unsigned char>(std::min(253.0, cost));
  }

  return FREE_SPACE;
}

}  // namespace costmap_plugin

// ============================================================
// pluginlib export
// ============================================================
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(costmap_plugin::RoughnessCostMap, costmap_plugin::CostMapBase)
