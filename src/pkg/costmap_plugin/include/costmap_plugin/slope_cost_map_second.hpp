#ifndef COSTMAP_PLUGIN__SLOPE_COST_MAP_HPP_
#define COSTMAP_PLUGIN__SLOPE_COST_MAP_HPP_

#include "costmap_plugin/cost_map_base.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include <vector>

namespace costmap_plugin
{

/**
 * @brief Cost map based on terrain slope.
 *
 * Computes traversability costs from 3D terrain data.
 * Supports two data sources:
 *   1. Point cloud (preferred): elevation map is built externally by
 *      GradientLayer and passed via setElevationMap()
 *   2. OctoMap (fallback): elevation map is built internally from
 *      OctoMap voxels via rebuildElevationMap()
 *
 * For each costmap cell, it computes the slope angle using centered
 * finite differences (CEAS_AIDAA paper, eq. 1-2).
 *
 * Parameters (yaml):
 *   - slope_threshold_deg  : maximum traversable slope in degrees
 *   - cost_scaling_factor  : multiplier to map slope -> cost value
 *   - search_radius        : radius (meters) around the robot to process voxels
 *   - weight               : contribution weight in the combined cost
 */
class SlopeCostMap : public CostMapBase
{
public:
  SlopeCostMap() = default;
  ~SlopeCostMap() override = default;

  void initialize(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const std::string & name) override;

  unsigned char computeCost(
    unsigned int mx,
    unsigned int my,
    const nav2_costmap_2d::Costmap2D & master_grid,
    std::shared_ptr<octomap::ColorOcTree> octree) override;

  /**
   * @brief Returns true if elevation data is available from either source.
   */
  bool isReady() const override { return has_elevation_data_ || octree_ != nullptr; }

  void setOctree(std::shared_ptr<octomap::ColorOcTree> octree) { octree_ = octree; }

  /**
   * @brief Receive an externally-built elevation map (from point cloud).
   *        This is the preferred data path - bypasses OctoMap entirely.
   */
  void setElevationMap(
    const std::vector<float> & elevation_map,
    unsigned int size_x,
    unsigned int size_y,
    double resolution);

  /**
   * @brief Rebuild the internal elevation map from OctoMap (fallback).
   */
  void rebuildElevationMap(
    const nav2_costmap_2d::Costmap2D & master_grid,
    double robot_wx,
    double robot_wy);

private:
  void buildElevationMap(
    const nav2_costmap_2d::Costmap2D & master_grid,
    double robot_wx,
    double robot_wy);

  double computeSlopeDeg(
    unsigned int mx,
    unsigned int my,
    const nav2_costmap_2d::Costmap2D & master_grid) const;

  // OctoMap reference (fallback)
  std::shared_ptr<octomap::ColorOcTree> octree_;

  // Elevation map: one float per costmap cell
  std::vector<float> elevation_map_;
  static constexpr float INVALID_ELEVATION = -1000.0f;

  // Flag: true when elevation data has been set externally
  bool has_elevation_data_ = false;

  // Parameters
  double slope_threshold_deg_ = 15.0;
  double cost_scaling_factor_ = 10.0;
  double search_radius_       = 10.0;

  // Cached costmap dimensions
  unsigned int size_x_ = 0;
  unsigned int size_y_ = 0;
  double resolution_   = 0.05;
};

}  // namespace costmap_plugin

#endif  // COSTMAP_PLUGIN__SLOPE_COST_MAP_HPP_
