#ifndef COSTMAP_PLUGIN__COST_MAP_BASE_HPP_
#define COSTMAP_PLUGIN__COST_MAP_BASE_HPP_

#include <string>
#include <vector>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include <octomap/ColorOcTree.h>

namespace costmap_plugin
{

/**
 * @brief Abstract base interface for cost maps.
 *
 * Every cost map (slope, shadow, roughness, etc.) must inherit from
 * this class and implement initialize() and computeCost().
 *
 * GradientLayer loads N instances of CostMapBase via pluginlib,
 * calls computeCost() for each cell, and combines the results
 * using a weighted sum.
 */
class CostMapBase
{
public:
  virtual ~CostMapBase() = default;

  /**
   * @brief Initialize the cost map with the parent ROS2 lifecycle node.
   *        Used to declare/read parameters and create subscribers.
   * @param node  Parent lifecycle node (Nav2 costmap layers use LifecycleNode)
   * @param name  Unique name used as parameter namespace prefix
   */
  virtual void initialize(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const std::string & name) = 0;

  /**
   * @brief Compute the cost for a single costmap cell.
   * @param mx          Cell x coordinate in the costmap grid
   * @param my          Cell y coordinate in the costmap grid
   * @param master_grid Reference to the master costmap
   * @param octree      Current 3D OctoMap (may be nullptr if not needed)
   * @return            Cost in [0, 254], or LETHAL_OBSTACLE (255)
   */
  virtual unsigned char computeCost(
    unsigned int mx,
    unsigned int my,
    const nav2_costmap_2d::Costmap2D & master_grid,
    std::shared_ptr<octomap::ColorOcTree> octree) = 0;

  /**
   * @brief Returns true if this cost map has valid data ready.
   *        GradientLayer skips cost maps that are not ready.
   */
  virtual bool isReady() const = 0;

  /**
   * @brief Receive shared elevation data built by GradientLayer.
   *
   * Default implementation is a no-op. Cost maps that need elevation
   * data (e.g., SlopeCostMap, RoughnessCostMap) should override this.
   * Cost maps that don't need it (e.g., ShadowCostMap) can ignore it.
   *
   * @param elevation_map  Flat array of elevation values per costmap cell
   * @param size_x         Costmap width in cells
   * @param size_y         Costmap height in cells
   * @param resolution     Costmap resolution in meters/cell
   */
  virtual void setElevationData(
    const std::vector<float> & elevation_map,
    unsigned int size_x,
    unsigned int size_y,
    double resolution)
  {
    (void)elevation_map;
    (void)size_x;
    (void)size_y;
    (void)resolution;
  }

  /**
   * @brief Rebuild internal data from OctoMap (fallback when no cloud).
   *
   * Default implementation is a no-op. Override if the cost map
   * can derive its data from an OctoMap.
   */
  virtual void rebuildFromOctree(
    const nav2_costmap_2d::Costmap2D & master_grid,
    double robot_wx,
    double robot_wy,
    std::shared_ptr<octomap::ColorOcTree> octree)
  {
    (void)master_grid;
    (void)robot_wx;
    (void)robot_wy;
    (void)octree;
  }

  /**
   * @brief Returns the name of this cost map.
   */
  const std::string & getName() const { return name_; }

  /**
   * @brief Returns the weight of this cost map in the combined result.
   *        Configurable from yaml as: weight: 1.0
   */
  double getWeight() const { return weight_; }

  /**
   * @brief Sets the weight of this cost map.
   */
  void setWeight(double weight) { weight_ = weight; }

protected:
  std::string name_;
  double weight_ = 1.0;
  rclcpp::Logger logger_ = rclcpp::get_logger("cost_map_base");
};

}  // namespace costmap_plugin

#endif  // COSTMAP_PLUGIN__COST_MAP_BASE_HPP_
