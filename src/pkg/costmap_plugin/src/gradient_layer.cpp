#include "gradient_layer.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <limits>
#include <algorithm>

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;

namespace costmap_plugin
{

// ============================================================
// Constructor / Destructor
// ============================================================

GradientLayer::GradientLayer()
: last_min_x_(std::numeric_limits<double>::lowest()),
  last_min_y_(std::numeric_limits<double>::lowest()),
  last_max_x_(std::numeric_limits<double>::max()),
  last_max_y_(std::numeric_limits<double>::max())
{
  access_ = new mutex_t();
}

GradientLayer::~GradientLayer()
{
  auto node = node_.lock();
  if (dyn_params_handler_ && node) {
    node->remove_on_set_parameters_callback(dyn_params_handler_.get());
  }
  dyn_params_handler_.reset();

  // IMPORTANT: destroy cost_maps_ BEFORE the ClassLoader
  cost_maps_.clear();
  cost_map_loader_.reset();

  delete access_;
}

// ============================================================
// onInitialize
// ============================================================

void GradientLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // --- Layer parameters ---
  declareParameter("enabled",            rclcpp::ParameterValue(true));
  declareParameter("octomap_topic",      rclcpp::ParameterValue("/rtabmap/octomap_full"));
  declareParameter("cloud_topic",        rclcpp::ParameterValue(""));
  declareParameter("search_radius",      rclcpp::ParameterValue(25.0));
  declareParameter("cost_map_plugins",   rclcpp::ParameterValue(std::vector<std::string>{}));

  node->get_parameter(name_ + ".enabled",       enabled_);
  node->get_parameter(name_ + ".octomap_topic", octomap_topic_);
  node->get_parameter(name_ + ".cloud_topic",   cloud_topic_);
  node->get_parameter(name_ + ".search_radius", search_radius_);

  // --- OctoMap subscriber (fallback, kept for compatibility) ---
  rclcpp::QoS octomap_qos(2);
  octomap_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  octomap_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);

  octomap_sub_ = node->create_subscription<octomap_msgs::msg::Octomap>(
    octomap_topic_, octomap_qos,
    std::bind(&GradientLayer::octomapCallback, this, std::placeholders::_1));

  // --- Point cloud subscriber (primary data source) ---
  if (!cloud_topic_.empty()) {
    rclcpp::QoS cloud_qos(2);
    cloud_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    cloud_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);

    cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, cloud_qos,
      std::bind(&GradientLayer::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(logger_, "GradientLayer: subscribing to cloud topic: %s", cloud_topic_.c_str());
  }

  // --- Debug publishers ---
  elevation_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "gradient_layer/elevation_map", rclcpp::QoS(1).transient_local());
  marker_pub_ = node->create_publisher<visualization_msgs::msg::Marker>(
    "debug/footprint_cells", 10);

  // ----------------------------------------------------------------
  // Load cost map plugins dynamically via pluginlib
  // ----------------------------------------------------------------

  cost_map_loader_ = std::make_shared<pluginlib::ClassLoader<CostMapBase>>(
    "costmap_plugin", "costmap_plugin::CostMapBase");

  std::vector<std::string> cost_map_plugin_names;
  node->get_parameter(name_ + ".cost_map_plugins", cost_map_plugin_names);

  for (const auto & plugin_name : cost_map_plugin_names) {
    std::string plugin_type;
    try {
      declareParameter(plugin_name + ".plugin", rclcpp::ParameterValue(""));
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {}
    node->get_parameter(name_ + "." + plugin_name + ".plugin", plugin_type);

    if (plugin_type.empty()) {
      RCLCPP_ERROR(logger_,
        "GradientLayer: cost map '%s' has no 'plugin' parameter, skipping",
        plugin_name.c_str());
      continue;
    }

    try {
      auto cost_map = cost_map_loader_->createSharedInstance(plugin_type);
      cost_map->initialize(node, name_ + "." + plugin_name);
      cost_maps_.push_back(cost_map);
      cost_map_names_.push_back(plugin_name);

      // Create a publisher for this cost map's individual costs
      std::string topic_name = "gradient_layer/" + plugin_name + "_costs";
      auto pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
        topic_name, rclcpp::QoS(1).transient_local());
      cost_map_pubs_.push_back(pub);

      RCLCPP_WARN(logger_, "GradientLayer: loaded cost map plugin '%s' (type: %s), publishing on '%s'",
                  plugin_name.c_str(), plugin_type.c_str(), topic_name.c_str());
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(logger_,
        "GradientLayer: failed to load cost map plugin '%s' (type: %s): %s",
        plugin_name.c_str(), plugin_type.c_str(), ex.what());
    }
  }

  RCLCPP_WARN(logger_, "GradientLayer initialized with %zu cost map(s), cloud_topic='%s'",
              cost_maps_.size(), cloud_topic_.c_str());

  current_ = true;
  need_regradient_ = false;
  matchSize();
}

// ============================================================
// octomapCallback (fallback)
// ============================================================

void GradientLayer::octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
  std::lock_guard<mutex_t> guard(*getMutex());

  octomap::AbstractOcTree * tree = octomap_msgs::fullMsgToMap(*msg);
  if (!tree) return;

  auto color_tree = dynamic_cast<octomap::ColorOcTree *>(tree);
  if (color_tree) {
    octree_ = std::make_shared<octomap::ColorOcTree>(*color_tree);
    if (!cloud_received_) {
      need_regradient_ = true;
    }
  }
  delete tree;
}

// ============================================================
// cloudCallback (primary data source)
// ============================================================

void GradientLayer::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::lock_guard<mutex_t> guard(*getMutex());

  RCLCPP_WARN(logger_, "GradientLayer: cloudCallback ENTERED, msg size=%u x %u, point_step=%u",
              msg->width, msg->height, msg->point_step);

  cloud_points_.clear();

  try {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      float x = *iter_x;
      float y = *iter_y;
      float z = *iter_z;

      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

      cloud_points_.push_back({x, y, z});
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "GradientLayer: cloudCallback EXCEPTION: %s", e.what());
    return;
  }

  cloud_received_ = true;
  need_regradient_ = true;

  RCLCPP_WARN(logger_, "GradientLayer: received cloud with %zu valid points", cloud_points_.size());
}

// ============================================================
// buildElevationFromCloud
// ============================================================

void GradientLayer::buildElevationFromCloud(
  const nav2_costmap_2d::Costmap2D & master_grid,
  double robot_wx, double robot_wy)
{
  unsigned int size_x = master_grid.getSizeInCellsX();
  unsigned int size_y = master_grid.getSizeInCellsY();

  elevation_map_.assign(size_x * size_y, INVALID_ELEVATION);

  if (cloud_points_.empty()) return;

  double robot_wz = 0.0;
  {
    std::vector<float> nearby_z;
    for (const auto & pt : cloud_points_) {
      double dx = pt[0] - robot_wx;
      double dy = pt[1] - robot_wy;
      if (dx * dx + dy * dy < 1.0) {
        nearby_z.push_back(pt[2]);
      }
    }
    if (!nearby_z.empty()) {
      std::sort(nearby_z.begin(), nearby_z.end());
      robot_wz = nearby_z[nearby_z.size() / 2];
    }
  }

  size_t used = 0;
  for (const auto & pt : cloud_points_) {
    double dx = pt[0] - robot_wx;
    double dy = pt[1] - robot_wy;
    double dz = pt[2] - robot_wz;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (dist > search_radius_) continue;

    unsigned int mx, my;
    if (!master_grid.worldToMap(pt[0], pt[1], mx, my)) continue;

    unsigned int idx = master_grid.getIndex(mx, my);
    if (idx >= elevation_map_.size()) continue;

    if (pt[2] > elevation_map_[idx]) {
      elevation_map_[idx] = pt[2];
    }
    ++used;
  }

  RCLCPP_INFO_THROTTLE(logger_, *node_.lock()->get_clock(), 10000,
    "GradientLayer: elevation map built from %zu/%zu cloud points, robot_wz=%.2f",
    used, cloud_points_.size(), robot_wz);
}

// ============================================================
// matchSize
// ============================================================

void GradientLayer::matchSize()
{
  std::lock_guard<mutex_t> guard(*getMutex());
  nav2_costmap_2d::Costmap2D * costmap = layered_costmap_->getCostmap();
  resolution_ = costmap->getResolution();
}

// ============================================================
// updateBounds
// ============================================================

void GradientLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<mutex_t> guard(*getMutex());

  last_pose_x_   = robot_x;
  last_pose_y_   = robot_y;
  last_pose_yaw_ = robot_yaw;

  if (need_regradient_) {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;

    *min_x = std::numeric_limits<double>::lowest();
    *min_y = std::numeric_limits<double>::lowest();
    *max_x = std::numeric_limits<double>::max();
    *max_y = std::numeric_limits<double>::max();

    need_regradient_ = false;
  } else {
    double tmp_min_x = last_min_x_, tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_, tmp_max_y = last_max_y_;

    last_min_x_ = *min_x; last_min_y_ = *min_y;
    last_max_x_ = *max_x; last_max_y_ = *max_y;

    *min_x = std::min(tmp_min_x, *min_x);
    *min_y = std::min(tmp_min_y, *min_y);
    *max_x = std::max(tmp_max_x, *max_x);
    *max_y = std::max(tmp_max_y, *max_y);
  }
}

// ============================================================
// onFootprintChanged
// ============================================================

void GradientLayer::onFootprintChanged()
{
  std::lock_guard<mutex_t> guard(*getMutex());
  footprint_ = layered_costmap_->getFootprint();
  need_regradient_ = true;
}

// ============================================================
// combineCosts
// ============================================================

unsigned char GradientLayer::combineCosts(
  unsigned int mx,
  unsigned int my,
  const nav2_costmap_2d::Costmap2D & master_grid)
{
  double total_weight = 0.0;
  double weighted_sum = 0.0;

  for (auto & cost_map : cost_maps_) {
    if (!cost_map->isReady()) continue;

    unsigned char cost = cost_map->computeCost(mx, my, master_grid, octree_);

    if (cost == LETHAL_OBSTACLE) {
      return LETHAL_OBSTACLE;
    }

    double w = cost_map->getWeight();
    weighted_sum  += w * static_cast<double>(cost);
    total_weight  += w;
  }

  if (total_weight == 0.0) return FREE_SPACE;

  double combined = weighted_sum / total_weight;
  return static_cast<unsigned char>(std::min(253.0, combined));
}

// ============================================================
// publishIndividualCostMaps
// ============================================================

void GradientLayer::publishIndividualCostMaps(
  const nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  unsigned int size_x = master_grid.getSizeInCellsX();
  unsigned int size_y = master_grid.getSizeInCellsY();

  for (size_t k = 0; k < cost_maps_.size(); ++k) {
    if (!cost_maps_[k]->isReady()) continue;
    if (k >= cost_map_pubs_.size()) continue;

    // Check if anyone is subscribed
    if (cost_map_pubs_[k]->get_subscription_count() == 0) continue;

    nav_msgs::msg::OccupancyGrid grid;
    //grid.header.frame_id = master_grid.getGlobalFrameID();
    grid.header.frame_id = layered_costmap_->getGlobalFrameID();
    grid.header.stamp = node_.lock()->now();
    grid.info.resolution = master_grid.getResolution();
    grid.info.width = size_x;
    grid.info.height = size_y;
    grid.info.origin.position.x = master_grid.getOriginX();
    grid.info.origin.position.y = master_grid.getOriginY();
    grid.info.origin.position.z = 0.0;

    grid.data.assign(size_x * size_y, -1);  // -1 = unknown

    for (int j = min_j; j < max_j; ++j) {
      for (int i = min_i; i < max_i; ++i) {
        unsigned char cost = cost_maps_[k]->computeCost(i, j, master_grid, octree_);

        unsigned int idx = master_grid.getIndex(i, j);
        if (idx >= grid.data.size()) continue;

        // Map costmap values (0-255) to OccupancyGrid values (0-100)
        if (cost == LETHAL_OBSTACLE) {
          grid.data[idx] = 100;
        } else if (cost == FREE_SPACE) {
          grid.data[idx] = 0;
        } else {
          grid.data[idx] = static_cast<int8_t>(
            std::min(99.0, static_cast<double>(cost) * 100.0 / 253.0));
        }
      }
    }

    cost_map_pubs_[k]->publish(grid);
  }
}

// ============================================================
// updateCosts
// ============================================================

void GradientLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<mutex_t> guard(*getMutex());

  if (!enabled_) return;

  bool have_cloud = cloud_received_ && !cloud_points_.empty();
  bool have_octree = (octree_ != nullptr);

  if (!have_cloud && !have_octree) {
    RCLCPP_WARN_THROTTLE(logger_, *node_.lock()->get_clock(), 5000,
                         "GradientLayer: No data available yet (no cloud, no octree)");
    return;
  }

  unsigned int robot_mx, robot_my;
  if (!master_grid.worldToMap(last_pose_x_, last_pose_y_, robot_mx, robot_my)) {
    RCLCPP_WARN(logger_, "GradientLayer: Robot is outside costmap bounds");
    return;
  }

  unsigned int size_x = master_grid.getSizeInCellsX();
  unsigned int size_y = master_grid.getSizeInCellsY();
  double res = master_grid.getResolution();

  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);

  // --- Pass shared data to all cost maps via the generic interface ---
  if (have_cloud) {
    buildElevationFromCloud(master_grid, last_pose_x_, last_pose_y_);
    for (auto & cost_map : cost_maps_) {
      cost_map->setElevationData(elevation_map_, size_x, size_y, res);
    }
  } else if (have_octree) {
    for (auto & cost_map : cost_maps_) {
      cost_map->rebuildFromOctree(master_grid, last_pose_x_, last_pose_y_, octree_);
    }
  }

  // --- Publish individual cost maps for visualization ---
  publishIndividualCostMaps(master_grid, min_i, min_j, max_i, max_j);

  // Compute footprint cells
  std::unordered_set<unsigned int> footprint_cells;
  for (const auto & pt : footprint_) {
    unsigned int fx, fy;
    if (master_grid.worldToMap(pt.x, pt.y, fx, fy)) {
      footprint_cells.insert(master_grid.getIndex(fx, fy));
    }
  }

  unsigned char * master_array = master_grid.getCharMap();

  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      unsigned int index = master_grid.getIndex(i, j);
      unsigned char combined = combineCosts(i, j, master_grid);

      if (combined == LETHAL_OBSTACLE) {
        if (footprint_cells.count(index) > 0) {
          master_array[index] = FREE_SPACE;
        } else {
          master_array[index] = LETHAL_OBSTACLE;
        }
      } else {
        master_array[index] = std::max(master_array[index], combined);
      }
    }
  }

  current_ = true;
}

}  // namespace costmap_plugin

PLUGINLIB_EXPORT_CLASS(costmap_plugin::GradientLayer, nav2_costmap_2d::Layer)