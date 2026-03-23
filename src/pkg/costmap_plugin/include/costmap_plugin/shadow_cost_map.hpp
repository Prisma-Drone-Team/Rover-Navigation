#ifndef COSTMAP_PLUGIN__SHADOW_COST_MAP_HPP_
#define COSTMAP_PLUGIN__SHADOW_COST_MAP_HPP_

#include "costmap_plugin/cost_map_base.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <vector>
#include <array>

namespace costmap_plugin
{

/**
 * @brief Cost map based on light/shadow analysis.
 *
 * Computes whether each costmap cell is in shadow by ray-marching
 * along the sun direction on the elevation map. Cells in shadow
 * receive a configurable cost penalty.
 *
 * This is relevant for lunar surface navigation where:
 * - Shadowed regions have extreme temperature drops
 * - Solar-powered rovers need illuminated paths
 * - Sensor visibility is reduced in shadow
 *
 * The sun direction is configurable via YAML parameters,
 * allowing simulation of different times of day or lunar positions.
 *
 * Parameters (yaml):
 *   - sun_direction_x    : X component of sun direction vector
 *   - sun_direction_y    : Y component of sun direction vector
 *   - sun_direction_z    : Z component of sun direction vector (negative = downward)
 *   - shadow_cost        : cost assigned to cells in shadow [0-253]
 *   - max_ray_cells      : maximum ray length in grid cells
 *   - min_shadow_angle_deg : minimum sun elevation angle below which
 *                            everything is considered in shadow
 *   - weight             : contribution weight in the combined cost
 */
class ShadowCostMap : public CostMapBase
{
public:
  ShadowCostMap() = default;
  ~ShadowCostMap() override = default;

  void initialize(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const std::string & name) override;

  unsigned char computeCost(
    unsigned int mx,
    unsigned int my,
    const nav2_costmap_2d::Costmap2D & master_grid,
    std::shared_ptr<octomap::ColorOcTree> octree) override;

  bool isReady() const override { return has_elevation_data_; }

  void setElevationData(
    const std::vector<float> & elevation_map,
    unsigned int size_x,
    unsigned int size_y,
    double resolution) override;

private:
  /**
   * @brief Determine if a cell is in shadow via ray marching.
   *
   * From the cell, march along the 2D projection of the sun direction
   * on the elevation map. At each step, check if the terrain is high
   * enough to block the sun ray at that distance.
   *
   * @return true if the cell is in shadow, false if illuminated
   */
  bool isInShadow(unsigned int mx, unsigned int my) const;

  // Elevation map (received from GradientLayer)
  std::vector<float> elevation_map_;
  static constexpr float INVALID_ELEVATION = -1000.0f;
  bool has_elevation_data_ = false;

  // Pre-computed shadow map (one bool per cell, rebuilt on each setElevationData)
  std::vector<bool> shadow_map_;
  bool shadow_map_valid_ = false;

  // Cached costmap dimensions
  unsigned int size_x_ = 0;
  unsigned int size_y_ = 0;
  double resolution_   = 0.05;

  // Sun direction (points FROM sun TO ground, i.e., light direction)
  // Default: sun from upper-left, shining down-right
  double sun_dir_x_ = -0.5;
  double sun_dir_y_ =  0.5;
  double sun_dir_z_ = -1.0;

  // Normalized 2D direction and vertical ratio for ray marching
  double ray_dx_ = 0.0;   // normalized step in X (grid cells)
  double ray_dy_ = 0.0;   // normalized step in Y (grid cells)
  double ray_dz_ = 0.0;   // elevation gain per horizontal cell step (meters)

  // Parameters
  unsigned char shadow_cost_ = 200;  // cost for shadowed cells
  int max_ray_cells_         = 50;   // max ray marching distance in cells
  double min_sun_angle_deg_  = 5.0;  // below this, everything is shadow
};

}  // namespace costmap_plugin

#endif  // COSTMAP_PLUGIN__SHADOW_COST_MAP_HPP_
