#ifndef COSTMAP_PLUGIN__ROUGHNESS_COST_MAP_HPP_
#define COSTMAP_PLUGIN__ROUGHNESS_COST_MAP_HPP_

#include "costmap_plugin/cost_map_base.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <vector>

namespace costmap_plugin
{

/**
 * @brief Cost map based on terrain roughness (local elevation variance).
 *
 * For each costmap cell, computes the standard deviation of elevation
 * values in a local neighborhood (kernel). High variance indicates
 * rough/uneven terrain that may be difficult to traverse.
 *
 * This cost map reuses the shared elevation data built by GradientLayer
 * from the point cloud, received via setElevationData().
 *
 * Parameters (yaml):
 *   - roughness_threshold  : std deviation (meters) above which terrain is LETHAL
 *   - cost_scaling_factor  : multiplier to map roughness -> cost value
 *   - kernel_size          : neighborhood size (e.g., 3 = 3x3, 5 = 5x5 cells)
 *   - weight               : contribution weight in the combined cost
 */
class RoughnessCostMap : public CostMapBase
{
public:
  RoughnessCostMap() = default;
  ~RoughnessCostMap() override = default;

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
   * @brief Compute the standard deviation of elevation in a local neighborhood.
   * @return Standard deviation in meters, or 0.0 if insufficient data.
   */
  double computeRoughness(
    unsigned int mx,
    unsigned int my) const;

  // Elevation map (received from GradientLayer)
  std::vector<float> elevation_map_;
  static constexpr float INVALID_ELEVATION = -1000.0f;
  bool has_elevation_data_ = false;

  // Cached costmap dimensions
  unsigned int size_x_ = 0;
  unsigned int size_y_ = 0;
  double resolution_   = 0.05;

  // Parameters
  double roughness_threshold_  = 0.1;   // meters — above this = LETHAL
  double cost_scaling_factor_  = 800.0; // maps roughness (m) to cost [0-253]
  int    kernel_size_          = 3;     // 3 = 3x3 neighborhood
};

}  // namespace costmap_plugin

#endif  // COSTMAP_PLUGIN__ROUGHNESS_COST_MAP_HPP_
