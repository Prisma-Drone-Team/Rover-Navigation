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
 * Computes traversability costs from an OctoMap 3D occupancy tree.
 * For each costmap cell, it builds a local elevation map by projecting
 * OctoMap voxels onto the 2D grid, then computes the slope angle using
 * centered finite differences (CEAS_AIDAA paper, eq. 1-2).
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

  /**
   * @brief Initialize the cost map, reading ROS2 parameters.
   * @param node  Parent lifecycle node
   * @param name  Unique name for parameter namespacing
   */
  void initialize(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const std::string & name) override;

  /**
   * @brief Compute the slope-based cost for a single costmap cell.
   * @param mx          Cell x coordinate in the costmap
   * @param my          Cell y coordinate in the costmap
   * @param master_grid Reference to the master costmap
   * @param octree      Shared pointer to the current ColorOcTree
   * @return            Cost in [0, 254] or LETHAL_OBSTACLE (255)
   */
  unsigned char computeCost(
    unsigned int mx,
    unsigned int my,
    const nav2_costmap_2d::Costmap2D & master_grid,
    std::shared_ptr<octomap::ColorOcTree> octree) override;

  /**
   * @brief Returns true if the OctoMap has been received at least once.
   */
  bool isReady() const override { return octree_ != nullptr; }

  /**
   * @brief Update the internal OctoMap reference.
   *        Called by GradientLayer whenever a new OctoMap message arrives.
   */
  void setOctree(std::shared_ptr<octomap::ColorOcTree> octree) { octree_ = octree; }

  /**
   * @brief Rebuild the internal elevation map from the current OctoMap.
   *        Must be called once per update cycle by GradientLayer,
   *        before iterating over cells with computeCost().
   * @param master_grid  Reference costmap
   * @param robot_wx     Robot world x position
   * @param robot_wy     Robot world y position
   */
  void rebuildElevationMap(
    const nav2_costmap_2d::Costmap2D & master_grid,
    double robot_wx,
    double robot_wy);

private:
  /**
   * @brief Build a local elevation map by projecting OctoMap voxels
   *        within search_radius_ onto the 2D costmap grid.
   */
  void buildElevationMap(
    const nav2_costmap_2d::Costmap2D & master_grid,
    double robot_wx,
    double robot_wy);

  /**
   * @brief Compute the slope angle in degrees for a single cell
   *        using centered finite differences on the elevation map.
   * @return Slope angle in degrees, or 0.0 if data unavailable
   */
  double computeSlopeDeg(
    unsigned int mx,
    unsigned int my,
    const nav2_costmap_2d::Costmap2D & master_grid) const;

  // OctoMap reference (shared with GradientLayer)
  std::shared_ptr<octomap::ColorOcTree> octree_;

  // Elevation map: one float per costmap cell
  std::vector<float> elevation_map_;
  static constexpr float INVALID_ELEVATION = -1000.0f;

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