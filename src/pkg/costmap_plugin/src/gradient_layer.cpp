#include "gradient_layer.hpp"

#include <limits>
#include <map>
#include <vector>
#include <algorithm>
#include <utility>

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/parameter_events_filter.hpp"


using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::FREE_SPACE;
using rcl_interfaces::msg::ParameterType;

namespace costmap_plugin
{

GradientLayer::GradientLayer()
: inflation_radius_(0),
  inscribed_radius_(0),
  cost_scaling_factor_(0),
  inflate_unknown_(false),
  inflate_around_unknown_(false),
  cell_inflation_radius_(0),
  cached_cell_inflation_radius_(0),
  resolution_(0),
  cache_length_(0),
  last_min_x_(std::numeric_limits<double>::lowest()),
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
  delete access_;
}

void
GradientLayer::onInitialize()
{
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("inflation_radius", rclcpp::ParameterValue(0.55));
  declareParameter("cost_scaling_factor", rclcpp::ParameterValue(10.0));
  declareParameter("slope_threshold_deg", rclcpp::ParameterValue(15.0));
  {
    auto node = node_.lock();
    if (!node) {
      throw std::runtime_error{"Failed to lock node"};
    }
    node->get_parameter(name_ + "." + "enabled", enabled_);
    node->get_parameter(name_ + "." + "inflation_radius", inflation_radius_);
    node->get_parameter(name_ + "." + "cost_scaling_factor", cost_scaling_factor_);
    node->get_parameter(name_ + "." + "slope_threshold_deg", slope_threshold_deg_);

    octomap_sub_ = node->create_subscription<octomap_msgs::msg::Octomap>(
      "/rtab1/octomap_full", 2,
      std::bind(&GradientLayer::pointCloudCallback, this, std::placeholders::_1));
  }

  current_ = true;
  seen_.clear();
  cached_distances_.clear();
  cached_costs_.clear();
  need_regradient_ = false;
  cell_inflation_radius_ = cellDistance(inflation_radius_);
  auto node = node_.lock();
  elevation_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
  "gradient_layer/elevation_map", rclcpp::QoS(1).transient_local());
  filtered_octomap_pub_ = node->create_publisher<octomap_msgs::msg::Octomap>("filtered_octomap", rclcpp::QoS(1).transient_local());
  marker_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("debug/footprint_cells", 10);
  matchSize();
}

/**
 * @brief Adjust the internal data structures to match the size of the master costmap.
 */

void GradientLayer::pointCloudCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());
  // RCLCPP_INFO(logger_, "OCTOMAP received, converting to octree...");
  // Converti in Octree
  octomap::AbstractOcTree* tree = octomap_msgs::fullMsgToMap(*msg);
  if (tree) {
  auto color_tree = dynamic_cast<octomap::ColorOcTree*>(tree);
  if (color_tree) {
    octree_ = std::make_shared<octomap::ColorOcTree>(*color_tree);
    need_regradient_ = true;
    }
    delete tree;
  }
}

void
GradientLayer::matchSize()
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());
  nav2_costmap_2d::Costmap2D * costmap = layered_costmap_->getCostmap();
  resolution_ = costmap->getResolution();
  cell_inflation_radius_ = cellDistance(inflation_radius_);
  // computeCaches();
  // seen_ = std::vector<bool>(costmap->getSizeInCellsX() * costmap->getSizeInCellsY(), false);
}

void
GradientLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());
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
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = std::min(tmp_min_x, *min_x);
    *min_y = std::min(tmp_min_y, *min_y);
    *max_x = std::max(tmp_max_x, *max_x);
    *max_y = std::max(tmp_max_y, *max_y);
  }
  last_pose_x_= robot_x;
  last_pose_y_= robot_y;
  last_pose_yaw_ = robot_yaw;
  // RCLCPP_INFO(logger_, "Posa Robot: X: %f Y: %f", robot_x, robot_y);
  
}

/**
 * @brief Callback triggered when the robot's footprint changes.
 */
void
GradientLayer::onFootprintChanged()
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());
  inscribed_radius_ = layered_costmap_->getInscribedRadius();
  cell_inflation_radius_ = cellDistance(inflation_radius_);
  footprint = layered_costmap_->getFootprint();
  // computeCaches();
  need_regradient_ = true;
}


/**
 * @brief Update the master costmap with the gradient costs around obstacles.
 * @param min_i Minimum x index of the update bounds.
 * @param min_j Minimum y index of the update bounds.
 * @param max_i Maximum x index of the update bounds.
 * @param max_j Maximum y index of the update bounds.
 */
void
GradientLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());
  if (!enabled_ || !octree_) {
    RCLCPP_ERROR(logger_, "Octree pointer is mullptr");
    return;
  }
  // RCLCPP_INFO(logger_, "Update COST");

  unsigned char * master_array = master_grid.getCharMap();
  if (!master_array) {
  RCLCPP_ERROR(logger_, "master_array è nullptr");
  return;
  }
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();
  reference_z_map_.clear();
  seen_.clear();
  occupied_.clear();

  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);
  constexpr float INVALID_ELEVATION = std::numeric_limits<float>::lowest();

  reference_z_map_.resize(size_x * size_y, 0);
  seen_.resize(size_x * size_y, false);
  occupied_ = std::vector<bool>(size_x * size_y, false);
  for (unsigned int i = 0; i < size_x * size_y; ++i) {
    if(master_array[i] == nav2_costmap_2d::NO_INFORMATION) {
       reference_z_map_[i] = -1000; 
    }
  }
  RCLCPP_INFO(logger_, "Seen inizializzata");

  unsigned int robot_mx, robot_my;
  if (!master_grid.worldToMap(last_pose_x_,last_pose_y_ , robot_mx, robot_my)) {
    RCLCPP_WARN(logger_, "Robot is outside the costmap bounds");
    return;
  }
  double last_pose_z_ = 0.0; 

  // auto filtered_tree = filterLocalOctomap(master_grid, 1.0);
  auto filtered_tree = gradient_utils::filterOctomapWithinRadius(octree_, master_grid,reference_z_map_, occupied_,
  last_pose_x_, last_pose_y_, last_pose_z_, 5.0, min_dist);

  // if (gradient_utils::collisionCheck(octree_, last_pose_x_, last_pose_y_, last_pose_z_, 1.0)){
  //   RCLCPP_INFO(logger_, "Collision detected within 1 meter radius of the robot");
  // }


  // RCLCPP_INFO(logger_, "Filtered OctoMap has %lu leaf nodes", filtered_tree->getNumLeafNodes());
  // octomap_msgs::msg::Octomap msg;
  // octomap_msgs::fullMapToMsg(*filtered_tree, msg);
  // msg.header.frame_id = "rover/map";
  // msg.header.stamp = node_.lock()->now();
  // filtered_octomap_pub_->publish(msg);


  // RCLCPP_INFO(logger_, "Attitude map riempita");





  // Pubblicazione della mappa di elevazione
  // nav_msgs::msg::OccupancyGrid elevation_msg;
  // elevation_msg.header.stamp = node_.lock()->now();
  // elevation_msg.header.frame_id = layered_costmap_->getGlobalFrameID();
  // elevation_msg.info.resolution = master_grid.getResolution();
  // elevation_msg.info.width = master_grid.getSizeInCellsX();
  // elevation_msg.info.height = master_grid.getSizeInCellsY();
  // elevation_msg.info.origin.position.x = master_grid.getOriginX();
  // elevation_msg.info.origin.position.y = master_grid.getOriginY();
  // elevation_msg.info.origin.position.z = 0.0;
  // elevation_msg.info.origin.orientation.w = 1.0;

  // elevation_msg.data.resize(reference_z_map_.size());

  // for (size_t i = 0; i < reference_z_map_.size(); ++i) {
  //   float elevation = reference_z_map_[i];
  //   if (elevation < -100 || elevation > 100) {
  //     elevation_msg.data[i] = -1;  // no data
  //   } else {
  //     // Normalize for 0-100 display in RViz
  //     elevation_msg.data[i] = std::min(100, std::max(0, static_cast<int>(elevation * 100)));
  //   }
  // }

  // elevation_pub_->publish(elevation_msg);

  // Ottenimento celle corrispondenti al footprint del robot
  std::unordered_set<unsigned int> footprint_cells;
  footprint = layered_costmap_->getFootprint();
  for (const auto& pt : footprint) {
    unsigned int fx, fy;
    double global_x = std::cos(last_pose_yaw_) * pt.x - std::sin(last_pose_yaw_) * pt.y + last_pose_x_;
    double global_y = std::sin(last_pose_yaw_) * pt.x + std::cos(last_pose_yaw_) * pt.y + last_pose_y_;
    if (master_grid.worldToMap(global_x, global_y, fx, fy)) {
      // Convert inflation_radius (in metri) in celle
      unsigned int radius_cells = static_cast<unsigned int>(std::ceil(inflation_radius_/ master_grid.getResolution()));

      // Espandi attorno al punto (fx, fy)
      for (int dx = -static_cast<int>(radius_cells-2); dx <= static_cast<int>(radius_cells-2); ++dx) {
        for (int dy = -static_cast<int>(radius_cells-2); dy <= static_cast<int>(radius_cells-2); ++dy) {
          int nx = static_cast<int>(fx) + dx;
          int ny = static_cast<int>(fy) + dy;

          if (nx >= 0 && ny >= 0 && nx < static_cast<int>(master_grid.getSizeInCellsX()) && ny < static_cast<int>(master_grid.getSizeInCellsY())) {
            unsigned int nidx = master_grid.getIndex(nx, ny);
            footprint_cells.insert(nidx);
          }
        }
      }
    }
  }

    //Visulizzazione footprint esteso ottenuto
    // visualization_msgs::msg::Marker marker;
    // marker.header.frame_id = layered_costmap_->getGlobalFrameID();
    // marker.header.stamp = node_.lock()->now();
    // marker.ns = "footprint_debug";
    // marker.id = 0;
    // marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    // marker.action = visualization_msgs::msg::Marker::ADD;
    // marker.scale.x = master_grid.getResolution();
    // marker.scale.y = master_grid.getResolution();
    // marker.scale.z = 0.05;  // Altezza visiva

    // marker.color.r = 0.0f;
    // marker.color.g = 0.0f;
    // marker.color.b = 1.0f;  // Blu
    // marker.color.a = 0.6f;

    // for (const auto& idx : footprint_cells) {
    //   unsigned int mx = idx % master_grid.getSizeInCellsX();
    //   unsigned int my = idx / master_grid.getSizeInCellsX();
    //   double wx, wy;
    //   master_grid.mapToWorld(mx, my, wx, wy);

    //   geometry_msgs::msg::Point p;
    //   p.x = wx + master_grid.getResolution() / 2.0;
    //   p.y = wy + master_grid.getResolution() / 2.0;
    //   p.z = 0.05;  // Leggermente sollevato da terra
    //   marker.points.push_back(p);
    // }

    // marker_pub_->publish(marker);

  size_t cells_visited = 0;
  std::queue<std::pair<unsigned int, unsigned int>> q;
  q.emplace(robot_mx, robot_my);

  while (!q.empty()) {
    auto [mx, my] = q.front();
    q.pop();

    // if (++cells_visited > 500000) {
    //   break;
    // }    
    
    unsigned int index = master_grid.getIndex(mx, my);
    if (index >= size_x * size_y) {
      continue; // Skip out of bounds cells
    }
 
    if (seen_[index] || (master_array[index] == nav2_costmap_2d::NO_INFORMATION)) { //
      continue;
    }
    seen_[index] = true;

    
    const int dx[8] = {1, -1, 0, 0, 1, -1, 1, -1};
    const int dy[8] = {0, 0, 1, -1, 1, 1, -1, -1};
    float min_z = reference_z_map_[index], max_z = reference_z_map_[index];
    double wx, wy;
    master_grid.mapToWorld(mx, my, wx, wy);
    double min_x = wx, min_y = wy;
    double max_x = wx, max_y = wy;

    for (int dir = 0; dir < 8; ++dir) {
      int nx = static_cast<int>(mx) + dx[dir];
      int ny = static_cast<int>(my) + dy[dir];
      if (nx < static_cast<int>(size_x) && ny < static_cast<int>(size_y)) {
        unsigned int nidx = master_grid.getIndex(nx, ny);
        if (nidx >= reference_z_map_.size() || std::abs(reference_z_map_[nidx]) <= 0.1 || !occupied_[nidx]) {
          continue; // Skip out of bounds cells
        }
        else if (reference_z_map_[nidx] < min_z) {
            min_z = reference_z_map_[nidx];
            master_grid.mapToWorld(nx, ny, min_x, min_y);
        }
        else if (reference_z_map_[nidx] > max_z) {
            max_z = reference_z_map_[nidx];
            master_grid.mapToWorld(nx, ny, max_x, max_y);
        }
      }
    }
    double dz = static_cast<double>(max_z) - static_cast<double>(min_z);
    double delta_x = (max_x - min_x);
    double delta_y = (max_y - min_y);
    double dist = std::sqrt(delta_x * delta_x + delta_y * delta_y + 0.1*0.1);
    if (dist < 1e-6f) {
        dist = 1e-6f;
      }
    double slope_deg = 0.0f;
    if(abs(dz) < 1){
    slope_deg = std::atan2(dz,dist) * 180.0f / M_PI;
    }
    
    if (slope_deg > slope_threshold_deg_)
      {
      // RCLCPP_INFO(logger_, "Max Z: %f  Min Z: %f  Dz: %f  Slope deg: %f", max_z, min_z, dz, slope_deg);
      if (footprint_cells.count(index)) {
      master_array[index] = nav2_costmap_2d::FREE_SPACE;
        }
      else{ master_array[index] = nav2_costmap_2d::LETHAL_OBSTACLE;
        }
      }


    // Enqueue celle adiacenti
    // const int dx[4] = {1, -1, 0, 0};
    // const int dy[4] = {0, 0, 1, -1};
    for (int dir = 0; dir < 8; ++dir) {
      int nx = static_cast<int>(mx) + dx[dir];
      int ny = static_cast<int>(my) + dy[dir];
      if (nx < static_cast<int>(size_x) && ny < static_cast<int>(size_y)) {
        unsigned int nidx = master_grid.getIndex(nx, ny);
        if (nidx < seen_.size() && !seen_[nidx] ) { //&& (master_array[nidx] == nav2_costmap_2d::FREE_SPACE)
          q.emplace(nx, ny);
        }
      }
    }
  }
  current_ = true;
}

/**
 * @brief  Given an index of a cell in the costmap, place it into a list pending for obstacle gradient
 * @param  grid The costmap
 * @param  index The index of the cell
 * @param  mx The x coordinate of the cell (can be computed from the index, but saves time to store it)
 * @param  my The y coordinate of the cell (can be computed from the index, but saves time to store it)
 * @param  src_x The x index of the obstacle point gradient started at
 * @param  src_y The y index of the obstacle point gradient started at
 */
void
GradientLayer::enqueue(
  unsigned int index, unsigned int mx, unsigned int my,
  unsigned int src_x, unsigned int src_y)
{
  if (!seen_[index]) {
    // we compute our distance table one cell further than the
    // gradient radius dictates so we can make the check below
    double distance = distanceLookup(mx, my, src_x, src_y);

    // we only want to put the cell in the list if it is within
    // the gradient radius of the obstacle point
    if (distance > cell_inflation_radius_) {
      return;
    }

    const unsigned int r = cell_inflation_radius_ + 2;

    // push the cell data onto the gradient list and mark
    const auto dist = distance_matrix_[mx - src_x + r][my - src_y + r];
    gradient_cells_[dist].emplace_back(mx, my, src_x, src_y);
  }
}

void
GradientLayer::computeCaches()
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());
  if (cell_inflation_radius_ == 0) {
    return;
  }

  cache_length_ = cell_inflation_radius_ + 2;

  // based on the gradient radius... compute distance and cost caches
  if (cell_inflation_radius_ != cached_cell_inflation_radius_) {
    cached_costs_.resize(cache_length_ * cache_length_);
    cached_distances_.resize(cache_length_ * cache_length_);

    for (unsigned int i = 0; i < cache_length_; ++i) {
      for (unsigned int j = 0; j < cache_length_; ++j) {
        cached_distances_[i * cache_length_ + j] = hypot(i, j);
      }
    }

    cached_cell_inflation_radius_ = cell_inflation_radius_;
  }

  for (unsigned int i = 0; i < cache_length_; ++i) {
    for (unsigned int j = 0; j < cache_length_; ++j) {
      cached_costs_[i * cache_length_ + j] = computeCost(cached_distances_[i * cache_length_ + j]);
    }
  }

  int max_dist = generateIntegerDistances();
  gradient_cells_.clear();
  gradient_cells_.resize(max_dist + 1);
}

int
GradientLayer::generateIntegerDistances()
{
  const int r = cell_inflation_radius_ + 2;    // Gradient radius + 2, used to ensure the entire area is covered.
  const int size = r * 2 + 1;   // Size of the distance matrix

  std::vector<std::pair<int, int>> points;    // Vector to store all valid points (x, y) within the circular area.

  // Iterate through all points within a square region.
  for (int y = -r; y <= r; y++) {
    for (int x = -r; x <= r; x++) {
      // Only include points within the circle defined by the radius.
      if (x * x + y * y <= r * r) {
        points.emplace_back(x, y);
      }
    }
  }
  
  // Sort the points by distance from the origin (0, 0).
  std::sort(
    points.begin(), points.end(),
    [](const std::pair<int, int> & a, const std::pair<int, int> & b) -> bool {
      return a.first * a.first + a.second * a.second < b.first * b.first + b.second * b.second;
    }
  );

  // Create a distance matrix to store the computed distances.
  std::vector<std::vector<int>> distance_matrix(size, std::vector<int>(size, 0));
  std::pair<int, int> last = {0, 0};
  int level = 0;
  // Assign distance levels to each valid point.
  for (auto const & p : points) {
    if (p.first * p.first + p.second * p.second !=
      last.first * last.first + last.second * last.second)
    {
      level++;    // Increment the distance level when the distance changes.
    }
    // Store the level in the distance matrix.
    distance_matrix[p.first + r][p.second + r] = level;
    last = p;
  }

  // Save the distance matrix for later use.
  distance_matrix_ = distance_matrix;
  // Return the maximum distance value (level).
  return level;
}


// std::shared_ptr<octomap::OcTree> GradientLayer::filterLocalOctomap(
//   nav2_costmap_2d::Costmap2D & master_grid,
//   double radius_meters)
// {
//   auto filtered_tree = std::make_shared<octomap::OcTree>(octree_->getResolution());
//   double last_pose_z = 0.0;
//   min_dist = radius_meters;

//   for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
//     if (!octree_->isNodeOccupied(*it)) continue;

//     double wx = it.getX(), wy = it.getY(), wz = it.getZ();
//     double dist = std::sqrt(
//       std::pow(wx - last_pose_x_, 2) +
//       std::pow(wy - last_pose_y_, 2) +
//       std::pow(wz - last_pose_z, 2));

//     if (dist <= radius_meters && (wz <= -0.05 || wz >= 0.05)) {
//       filtered_tree->updateNode(octomap::point3d(wx, wy, wz), true);
//       if(dist <= min_dist){
//         min_dist=dist;
//       }
//     }

//     unsigned int mx, my;
//     if (master_grid.worldToMap(wx, wy, mx, my)) {
//       unsigned int idx = master_grid.getIndex(mx, my);
//       if (idx < reference_z_map_.size() && wz >= 0) {
//         reference_z_map_[idx] = wz;
//       }
//     }
//   }

//   filtered_tree->updateInnerOccupancy();
//   return filtered_tree;
// }

// double GradientLayer::getMinDist() const
// {
//   RCLCPP_INFO(logger_, "Nearest obstacle distance: %f", min_dist);
//   return min_dist;
// }

}  // namespace nav2_costmap_2d

PLUGINLIB_EXPORT_CLASS(costmap_plugin::GradientLayer, nav2_costmap_2d::Layer)