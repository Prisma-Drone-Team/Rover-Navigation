#ifndef COSTMAP_PLUGIN__GRADIENT_LAYER_HPP_
#define COSTMAP_PLUGIN__GRADIENT_LAYER_HPP_

#include <vector>
#include <map>
#include <queue>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>

#include "nav2_costmap_2d/layer.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/rclcpp.hpp"
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <pluginlib/class_loader.hpp>

// Cost map interface
#include "costmap_plugin/cost_map_base.hpp"

#include "octomap_utils.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::FREE_SPACE;
using rcl_interfaces::msg::ParameterType;

namespace costmap_plugin
{

class GradientLayer : public nav2_costmap_2d::Layer
{
public:
  GradientLayer();
  ~GradientLayer();

  virtual void onInitialize() override;
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y,
    double * max_x, double * max_y) override;
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  virtual void onFootprintChanged() override;
  virtual void matchSize() override;

  virtual void reset() override
  {
    RCLCPP_INFO(logger_, "Resetting GradientLayer");
    current_ = false;
  }

  virtual bool isClearable() override { return false; }

  typedef std::recursive_mutex mutex_t;
  mutex_t * getMutex() { return access_; }

private:
  // Callback for OctoMap (kept as fallback)
  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);

  // Callback for PointCloud2 from RTABMap cloud_map
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // Build elevation map from the stored point cloud
  void buildElevationFromCloud(
    const nav2_costmap_2d::Costmap2D & master_grid,
    double robot_wx, double robot_wy);

  // Combine costs from all registered cost maps
  unsigned char combineCosts(
    unsigned int mx,
    unsigned int my,
    const nav2_costmap_2d::Costmap2D & master_grid);

  // Publish individual cost maps as OccupancyGrid
  void publishIndividualCostMaps(
    const nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);
  
  // --- pluginlib loader for CostMapBase plugins ---
  std::shared_ptr<pluginlib::ClassLoader<CostMapBase>> cost_map_loader_;

  // Subscribers
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  // Debug publishers
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr elevation_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  // Per-plugin cost map publishers (name -> publisher)
  std::vector<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr> cost_map_pubs_;
  // Plugin names (parallel to cost_maps_ and cost_map_pubs_)
  std::vector<std::string> cost_map_names_;
  
  // Shared OctoMap (fallback)
  std::shared_ptr<octomap::ColorOcTree> octree_;

  // Stored point cloud data: vector of (x, y, z) points
  std::vector<std::array<float, 3>> cloud_points_;
  bool cloud_received_ = false;

  // Elevation map built from cloud
  std::vector<float> elevation_map_;
  static constexpr float INVALID_ELEVATION = -1000.0f;

  // List of registered cost maps (loaded via pluginlib)
  std::vector<std::shared_ptr<CostMapBase>> cost_maps_;

  // Footprint of the robot
  std::vector<geometry_msgs::msg::Point> footprint_;

  // Parameters
  bool enabled_           = true;
  std::string octomap_topic_;
  std::string cloud_topic_;
  double resolution_      = 0.05;
  double search_radius_   = 25.0;

  // Update bounds tracking
  bool need_regradient_   = false;
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
  double last_pose_x_ = 0.0, last_pose_y_ = 0.0, last_pose_yaw_ = 0.0;

  mutex_t * access_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

}  // namespace costmap_plugin

#endif  // COSTMAP_PLUGIN__GRADIENT_LAYER_HPP_