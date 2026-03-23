#include "costmap_plugin/slope_cost_map.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include <cmath>
#include <algorithm>

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;

namespace costmap_plugin
{

void SlopeCostMap::initialize(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::string & name)
{
  name_   = name;
  logger_ = rclcpp::get_logger(name_);

  node->declare_parameter(name_ + ".slope_threshold_deg", slope_threshold_deg_);
  node->declare_parameter(name_ + ".cost_scaling_factor", cost_scaling_factor_);
  node->declare_parameter(name_ + ".search_radius",       search_radius_);
  node->declare_parameter(name_ + ".weight",              weight_);

  node->get_parameter(name_ + ".slope_threshold_deg", slope_threshold_deg_);
  node->get_parameter(name_ + ".cost_scaling_factor", cost_scaling_factor_);
  node->get_parameter(name_ + ".search_radius",       search_radius_);
  node->get_parameter(name_ + ".weight",              weight_);

  RCLCPP_INFO(
    logger_,
    "[SlopeCostMap] initialized: threshold=%.1f deg, scaling=%.1f, radius=%.1f m, weight=%.2f",
    slope_threshold_deg_, cost_scaling_factor_, search_radius_, weight_);
}

// ============================================================
// setElevationData (from point cloud - preferred path)
// ============================================================

void SlopeCostMap::setElevationData(
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
// rebuildFromOctree (fallback - generic interface)
// ============================================================

void SlopeCostMap::rebuildFromOctree(
  const nav2_costmap_2d::Costmap2D & master_grid,
  double robot_wx,
  double robot_wy,
  std::shared_ptr<octomap::ColorOcTree> octree)
{
  octree_ = octree;
  buildElevationFromOctree(master_grid, robot_wx, robot_wy);
}

// ============================================================
// buildElevationFromOctree (internal - from OctoMap)
// ============================================================

void SlopeCostMap::buildElevationFromOctree(
  const nav2_costmap_2d::Costmap2D & master_grid,
  double robot_wx,
  double robot_wy)
{
  size_x_     = master_grid.getSizeInCellsX();
  size_y_     = master_grid.getSizeInCellsY();
  resolution_ = master_grid.getResolution();

  elevation_map_.assign(size_x_ * size_y_, INVALID_ELEVATION);

  if (!octree_) return;

  const double robot_wz = 0.0;

  for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
    if (!octree_->isNodeOccupied(*it)) continue;

    double wx = it.getX();
    double wy = it.getY();
    double wz = it.getZ();

    double dist = std::sqrt(
      std::pow(wx - robot_wx, 2) +
      std::pow(wy - robot_wy, 2) +
      std::pow(wz - robot_wz, 2));

    if (dist > search_radius_) continue;

    unsigned int mx, my;
    if (!master_grid.worldToMap(wx, wy, mx, my)) continue;

    unsigned int idx = master_grid.getIndex(mx, my);
    if (idx >= elevation_map_.size()) continue;

    if (wz > elevation_map_[idx]) {
      elevation_map_[idx] = static_cast<float>(wz);
    }
  }

  has_elevation_data_ = true;
}

// ============================================================
// computeSlopeDeg
// ============================================================

double SlopeCostMap::computeSlopeDeg(
  unsigned int mx,
  unsigned int my,
  const nav2_costmap_2d::Costmap2D & master_grid) const
{
  auto getElevation = [&](int ix, int iy, float & h_out) -> bool {
    if (ix < 0 || iy < 0 ||
        ix >= static_cast<int>(size_x_) ||
        iy >= static_cast<int>(size_y_))
    {
      return false;
    }
    float h = elevation_map_[master_grid.getIndex(ix, iy)];
    if (h <= INVALID_ELEVATION) return false;
    h_out = h;
    return true;
  };

  float h_px, h_mx, h_py, h_my;
  bool valid =
    getElevation(static_cast<int>(mx) + 1, static_cast<int>(my),     h_px) &&
    getElevation(static_cast<int>(mx) - 1, static_cast<int>(my),     h_mx) &&
    getElevation(static_cast<int>(mx),     static_cast<int>(my) + 1, h_py) &&
    getElevation(static_cast<int>(mx),     static_cast<int>(my) - 1, h_my);

  if (!valid) return 0.0;

  double dh_dx = (h_px - h_mx) / (2.0 * resolution_);
  double dh_dy = (h_py - h_my) / (2.0 * resolution_);
  double grad_norm = std::sqrt(dh_dx * dh_dx + dh_dy * dh_dy);

  if (grad_norm < 1e-3) return 0.0;

  return std::atan(grad_norm) * 180.0 / M_PI;
}

// ============================================================
// computeCost
// ============================================================

unsigned char SlopeCostMap::computeCost(
  unsigned int mx,
  unsigned int my,
  const nav2_costmap_2d::Costmap2D & master_grid,
  std::shared_ptr<octomap::ColorOcTree> octree)
{
  // Store octree reference for potential fallback use
  octree_ = octree;

  // If no elevation data has been set externally, try OctoMap fallback
  if (!has_elevation_data_ && octree_) {
    if (elevation_map_.size() != master_grid.getSizeInCellsX() * master_grid.getSizeInCellsY()) {
      double cx = master_grid.getOriginX() + master_grid.getSizeInCellsX() * resolution_ / 2.0;
      double cy = master_grid.getOriginY() + master_grid.getSizeInCellsY() * resolution_ / 2.0;
      buildElevationFromOctree(master_grid, cx, cy);
    }
  }

  if (!has_elevation_data_) return FREE_SPACE;

  double slope_deg = computeSlopeDeg(mx, my, master_grid);

  if (slope_deg >= slope_threshold_deg_) {
    return LETHAL_OBSTACLE;
  } else if (slope_deg > 1.0) {
    return static_cast<unsigned char>(
      std::min(253.0, cost_scaling_factor_ * slope_deg));
  }

  return FREE_SPACE;
}

}  // namespace costmap_plugin

// ============================================================
// pluginlib export for CostMapBase
// ============================================================
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(costmap_plugin::SlopeCostMap, costmap_plugin::CostMapBase)
