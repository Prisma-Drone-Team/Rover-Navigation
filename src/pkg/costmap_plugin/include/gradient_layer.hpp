#ifndef COSTMAP_PLUGIN__SLOPE_LAYER_HPP_
#define COSTMAP_PLUGIN__SLOPE_LAYER_HPP_

#include <vector>
#include <map>
#include <queue>
#include <memory>
#include <mutex>
#include <string>

#include "nav2_costmap_2d/layer.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/observation_buffer.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/rclcpp.hpp"
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>

#include "octomap_utils.hpp"

#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>


using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::FREE_SPACE;
using rcl_interfaces::msg::ParameterType;


namespace costmap_plugin
{

class CellData
{
public:
  /**
   * @brief  Constructor for a CellData objects
   * @param  x The x coordinate of the cell in the cost map
   * @param  y The y coordinate of the cell in the cost map
   * @param  sx The x coordinate of the closest obstacle cell in the costmap
   * @param  sy The y coordinate of the closest obstacle cell in the costmap
   * @return
   */
  CellData(unsigned int x, unsigned int y, unsigned int sx, unsigned int sy)
  : x_(x), y_(y), src_x_(sx), src_y_(sy)
  {
  }
  unsigned int x_, y_;
  unsigned int src_x_, src_y_;
};


class GradientLayer : public nav2_costmap_2d::Layer
{
public:
  GradientLayer();
  ~GradientLayer();
  virtual void onInitialize() override;
  virtual void updateBounds(double, double, double, double *, double *, double *, double *) override;
  virtual void updateCosts(nav2_costmap_2d::Costmap2D &, int, int, int, int) override;
  virtual void onFootprintChanged() override;

  virtual void reset() override{

    RCLCPP_INFO(logger_, "Resetting GradientLayer");

    current_ = false;
    first_iter = true;

    // reference_z_map_.clear();
    seen_.clear();
  }
  virtual bool isClearable() override {return false;}

  typedef std::recursive_mutex mutex_t;

  /**
   * @brief Get the mutex of the gradient information
   */
  mutex_t * getMutex()
  {
    return access_;
  }

  void matchSize() override;


protected:
  void pointCloudCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);
  float computeSlope(const std::vector<geometry_msgs::msg::Point>& points);

  unsigned int cellDistance(double world_dist)
  {
    return layered_costmap_->getCostmap()->cellDistance(world_dist);
  }
  void computeCaches();
  int generateIntegerDistances();
  void enqueue(
    unsigned int index, unsigned int mx, unsigned int my,
    unsigned int src_x, unsigned int src_y);

  inline double distanceLookup(
    unsigned int mx, unsigned int my, unsigned int src_x,
    unsigned int src_y)
  {
    unsigned int dx = (mx > src_x) ? mx - src_x : src_x - mx;
    unsigned int dy = (my > src_y) ? my - src_y : src_y - my;
    return cached_distances_[dx * cache_length_ + dy];
  }

  inline unsigned char computeCost(double distance) const
  {
    unsigned char cost = 0;
    if (distance == 0) {
      cost = LETHAL_OBSTACLE;
    } else if (distance * resolution_ <= inscribed_radius_) {
      cost = INSCRIBED_INFLATED_OBSTACLE;
    } else {
      // make sure cost falls off by Euclidean distance
      double factor =
        exp(-1.0 * cost_scaling_factor_ * (distance * resolution_ - inscribed_radius_));
      cost = static_cast<unsigned char>((INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
    }
    return cost;
  }

  // std::shared_ptr<octomap::OcTree> filterLocalOctomap(nav2_costmap_2d::Costmap2D & master_grid,double radius_meters);
  // double getMinDist() const;


  // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr elevation_pub_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr filtered_octomap_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  std::vector<geometry_msgs::msg::Point> lidar_points_;
  std::vector<geometry_msgs::msg::Point> footprint;
  std::mutex data_mutex_;
  std::shared_ptr<octomap::ColorOcTree> octree_;

  double inflation_radius_, inscribed_radius_, cost_scaling_factor_;
  bool inflate_unknown_, inflate_around_unknown_;
  unsigned int cell_inflation_radius_;
  unsigned int cached_cell_inflation_radius_;
  std::vector<std::vector<CellData>> gradient_cells_;
  std::vector<unsigned char> persistent_costs_;  // Costmap persistente
  bool first_update_;  // Flag per primo update
  std::string octomap_topic_;

  std::vector<float> reference_z_map_;
  std::vector<bool> seen_;
  std::vector<bool> occupied_;
  std::vector<unsigned char> cached_costs_;
  std::vector<double> cached_distances_;
  std::vector<std::vector<int>> distance_matrix_;
  unsigned int cache_length_, old_size_x_, old_size_y_;
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_, last_pose_x_, last_pose_y_, last_pose_yaw_;
  bool enabled_;
  double resolution_;
  bool first_iter=true;
  bool need_recalculation_;
  double slope_threshold_deg_, min_dist;
  bool need_regradient_;
  mutex_t * access_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr last_cloud_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

};

}  // namespace costmap_plugin

#endif  // COSTMAP_PLUGIN__SLOPE_LAYER_HPP_
