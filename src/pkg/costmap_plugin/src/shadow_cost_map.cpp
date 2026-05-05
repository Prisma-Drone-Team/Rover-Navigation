#include "costmap_plugin/shadow_cost_map.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include <cmath>
#include <algorithm>

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;

namespace costmap_plugin
{

void ShadowCostMap::initialize(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::string & name)
{
  name_   = name;
  logger_ = rclcpp::get_logger(name_);

  // Sun direction parameters
  node->declare_parameter(name_ + ".sun_direction_x", sun_dir_x_);
  node->declare_parameter(name_ + ".sun_direction_y", sun_dir_y_);
  node->declare_parameter(name_ + ".sun_direction_z", sun_dir_z_);

  // Cost parameters
  int shadow_cost_int = static_cast<int>(shadow_cost_);
  node->declare_parameter(name_ + ".shadow_cost",          shadow_cost_int);
  node->declare_parameter(name_ + ".max_ray_cells",        max_ray_cells_);
  node->declare_parameter(name_ + ".min_sun_angle_deg",    min_sun_angle_deg_);
  node->declare_parameter(name_ + ".weight",               weight_);

  node->get_parameter(name_ + ".sun_direction_x", sun_dir_x_);
  node->get_parameter(name_ + ".sun_direction_y", sun_dir_y_);
  node->get_parameter(name_ + ".sun_direction_z", sun_dir_z_);
  node->get_parameter(name_ + ".shadow_cost",     shadow_cost_int);
  node->get_parameter(name_ + ".max_ray_cells",   max_ray_cells_);
  node->get_parameter(name_ + ".min_sun_angle_deg", min_sun_angle_deg_);
  node->get_parameter(name_ + ".weight",           weight_);

  shadow_cost_ = static_cast<unsigned char>(
    std::min(253, std::max(0, shadow_cost_int)));

  // Pre-compute ray marching direction
  // The sun direction vector points from sun to ground (light direction).
  // To trace FROM a cell TOWARD the sun, invert the horizontal components.
  // We need the 2D horizontal direction (toward the sun) and the vertical
  // rise per horizontal unit (how much elevation the sun ray gains).

  double horiz_len = std::sqrt(sun_dir_x_ * sun_dir_x_ + sun_dir_y_ * sun_dir_y_);

  if (horiz_len < 1e-6) {
    // Sun is directly overhead — no shadows possible
    ray_dx_ = 0.0;
    ray_dy_ = 0.0;
    ray_dz_ = 0.0;
    RCLCPP_WARN(logger_,
      "[ShadowCostMap] Sun is directly overhead, no shadows will be computed");
  } else {
    // Direction toward the sun (inverted horizontal, since sun_dir points down)
    ray_dx_ = -sun_dir_x_ / horiz_len;
    ray_dy_ = -sun_dir_y_ / horiz_len;

    // Elevation gain per horizontal cell step:
    // For each cell step horizontally (distance = resolution),
    // the sun ray rises by (|sun_dir_z| / horiz_len) * resolution
    // But sun_dir_z is negative (pointing down), so we use -sun_dir_z
    ray_dz_ = (-sun_dir_z_ / horiz_len) * resolution_;
  }

  RCLCPP_WARN(
    logger_,
    "[ShadowCostMap] initialized: sun_dir=(%.2f, %.2f, %.2f), "
    "shadow_cost=%d, max_ray_cells=%d, min_angle=%.1f deg, weight=%.2f",
    sun_dir_x_, sun_dir_y_, sun_dir_z_,
    shadow_cost_, max_ray_cells_, min_sun_angle_deg_, weight_);
}

// ============================================================
// setElevationData
// ============================================================

void ShadowCostMap::setElevationData(
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

  // Recompute ray_dz with actual resolution (may differ from default)
  double horiz_len = std::sqrt(sun_dir_x_ * sun_dir_x_ + sun_dir_y_ * sun_dir_y_);
  if (horiz_len > 1e-6) {
    ray_dz_ = (-sun_dir_z_ / horiz_len) * resolution_;
  }

  // Invalidate shadow map — will be rebuilt lazily on first computeCost
  shadow_map_valid_ = false;
}

// ============================================================
// isInShadow — ray marching
// ============================================================

bool ShadowCostMap::isInShadow(unsigned int mx, unsigned int my) const
{
  // Check if sun is too low (nearly horizontal)
  double horiz_len = std::sqrt(sun_dir_x_ * sun_dir_x_ + sun_dir_y_ * sun_dir_y_);
  if (horiz_len < 1e-6) return false;  // sun overhead, no shadows

  double sun_elevation_deg = std::atan2(-sun_dir_z_, horiz_len) * 180.0 / M_PI;
  if (sun_elevation_deg < min_sun_angle_deg_) return true;  // sun too low

  // Get the elevation of the current cell
  unsigned int idx = my * size_x_ + mx;
  if (idx >= elevation_map_.size()) return false;

  float cell_elevation = elevation_map_[idx];
  if (cell_elevation <= INVALID_ELEVATION) return false;

  // March along the ray toward the sun
  // At each step, check if terrain blocks the sun ray
  double fx = static_cast<double>(mx);
  double fy = static_cast<double>(my);

  for (int step = 1; step <= max_ray_cells_; ++step) {
    fx += ray_dx_;
    fy += ray_dy_;

    int ix = static_cast<int>(std::round(fx));
    int iy = static_cast<int>(std::round(fy));

    // Out of bounds — ray escaped the map, cell is illuminated
    if (ix < 0 || iy < 0 ||
        ix >= static_cast<int>(size_x_) ||
        iy >= static_cast<int>(size_y_))
    {
      return false;
    }

    unsigned int ray_idx = iy * size_x_ + ix;
    if (ray_idx >= elevation_map_.size()) return false;

    float terrain_h = elevation_map_[ray_idx];
    if (terrain_h <= INVALID_ELEVATION) continue;  // no data, skip

    // Height the sun ray has at this horizontal distance from the cell
    // The ray starts at cell_elevation and rises by ray_dz_ per step
    double ray_h = cell_elevation + ray_dz_ * step;

    // If the terrain at this point is ABOVE the sun ray,
    // it blocks the light → the original cell is in shadow
    if (terrain_h > ray_h) {
      return true;
    }
  }

  // Ray reached max length without being blocked — cell is illuminated
  return false;
}

// ============================================================
// computeCost
// ============================================================

unsigned char ShadowCostMap::computeCost(
  unsigned int mx,
  unsigned int my,
  const nav2_costmap_2d::Costmap2D & /*master_grid*/,
  std::shared_ptr<octomap::ColorOcTree> /*octree*/)
{
  if (!has_elevation_data_) return FREE_SPACE;

  // Rebuild shadow map if needed (once per elevation update)
  if (!shadow_map_valid_) {
    shadow_map_.assign(size_x_ * size_y_, false);

    for (unsigned int y = 0; y < size_y_; ++y) {
      for (unsigned int x = 0; x < size_x_; ++x) {
        shadow_map_[y * size_x_ + x] = isInShadow(x, y);
      }
    }
    shadow_map_valid_ = true;

    // Count shadowed cells for debugging
    size_t shadow_count = 0;
    for (bool s : shadow_map_) { if (s) ++shadow_count; }

    // RCLCPP_WARN(
    //   logger_,
    //   "[ShadowCostMap] shadow map rebuilt: %zu/%zu cells in shadow (%.1f%%)",
    //   shadow_count, shadow_map_.size(),
    //   100.0 * shadow_count / std::max(shadow_map_.size(), size_t(1)));
  }

  unsigned int idx = my * size_x_ + mx;
  if (idx >= shadow_map_.size()) return FREE_SPACE;

  return shadow_map_[idx] ? shadow_cost_ : FREE_SPACE;
}

}  // namespace costmap_plugin

// ============================================================
// pluginlib export
// ============================================================
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(costmap_plugin::ShadowCostMap, costmap_plugin::CostMapBase)
