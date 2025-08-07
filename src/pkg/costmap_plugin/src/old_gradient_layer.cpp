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
: gradient_radius_(0),
  inscribed_radius_(0),
  cost_scaling_factor_(0),
  inflate_unknown_(false),
  inflate_around_unknown_(false),
  cell_gradient_radius_(0),
  cached_cell_gradient_radius_(0),
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
  declareParameter("gradient_radius", rclcpp::ParameterValue(0.55));
  declareParameter("cost_scaling_factor", rclcpp::ParameterValue(10.0));
  declareParameter("slope_threshold_deg", rclcpp::ParameterValue(15.0));
  {
    auto node = node_.lock();
    if (!node) {
      throw std::runtime_error{"Failed to lock node"};
    }
    node->get_parameter(name_ + "." + "enabled", enabled_);
    node->get_parameter(name_ + "." + "gradient_radius", gradient_radius_);
    node->get_parameter(name_ + "." + "cost_scaling_factor", cost_scaling_factor_);
    node->get_parameter(name_ + "." + "slope_threshold_deg", slope_threshold_deg_);

    octomap_sub_ = node->create_subscription<octomap_msgs::msg::Octomap>(
      "/octomap_full", 2,
      std::bind(&GradientLayer::pointCloudCallback, this, std::placeholders::_1));
  }

  current_ = true;
  seen_.clear();
  cached_distances_.clear();
  cached_costs_.clear();
  need_regradient_ = false;
  cell_gradient_radius_ = cellDistance(gradient_radius_);
  matchSize();
}

/**
 * @brief Adjust the internal data structures to match the size of the master costmap.
 */

void GradientLayer::pointCloudCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());
  RCLCPP_INFO(logger_, "OCTOMAP received, converting to octree...");
  // Converti in Octree
  octomap::AbstractOcTree* tree = octomap_msgs::fullMsgToMap(*msg);
  if (tree) {
  auto color_tree = dynamic_cast<octomap::ColorOcTree*>(tree);
  if (color_tree) {
    octree_ = std::make_shared<octomap::ColorOcTree>(*color_tree);
    need_regradient_ = true;
    } 
  }
}

void
GradientLayer::matchSize()
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());
  nav2_costmap_2d::Costmap2D * costmap = layered_costmap_->getCostmap();
  resolution_ = costmap->getResolution();
  cell_gradient_radius_ = cellDistance(gradient_radius_);
  computeCaches();
  seen_ = std::vector<bool>(costmap->getSizeInCellsX() * costmap->getSizeInCellsY(), false);
}

void
GradientLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
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
    double tmp_min_x = last_min_x_;   //The previous bounds are saved for the next cycle.
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = std::min(tmp_min_x, *min_x) - gradient_radius_;
    *min_y = std::min(tmp_min_y, *min_y) - gradient_radius_;
    *max_x = std::max(tmp_max_x, *max_x) + gradient_radius_;
    *max_y = std::max(tmp_max_y, *max_y) + gradient_radius_;
  }
}

/**
 * @brief Callback triggered when the robot's footprint changes.
 */
void
GradientLayer::onFootprintChanged()
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());
  inscribed_radius_ = layered_costmap_->getInscribedRadius();
  cell_gradient_radius_ = cellDistance(gradient_radius_);
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
  if (!enabled_) {
    return;
  }

  // make sure the gradient list is empty at the beginning of the cycle (should always be true)
  for (auto & dist : gradient_cells_) {
    RCLCPP_FATAL_EXPRESSION(
      logger_,
      !dist.empty(), "The gradient list must be empty at the beginning of gradient");
  }

  unsigned char * master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();
  if(first_iter){
    for (unsigned int y = 0; y < size_y; ++y) {
      for (unsigned int x = 0; x < size_x; ++x) {
        reference_z_map_[y * size_x + x] = 0;
      }
    }
    first_iter = false;
  }
  if (seen_.size() != size_x * size_y) {
    RCLCPP_WARN(
      logger_, "GradientLayer::updateCosts(): seen_ vector size is wrong");
    seen_ = std::vector<bool>(size_x * size_y, false);
  }

  std::fill(begin(seen_), end(seen_), false);   //Initializing a "seen" list to avoid reprocessing cells.

  const int base_min_i = min_i;
  const int base_min_j = min_j;
  const int base_max_i = max_i;
  const int base_max_j = max_j;
  // min_i -= static_cast<int>(cell_gradient_radius_);
  // min_j -= static_cast<int>(cell_gradient_radius_);
  // max_i += static_cast<int>(cell_gradient_radius_);
  // max_j += static_cast<int>(cell_gradient_radius_);

  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);

  // Start with lethal obstacles: by definition distance is 0.0
  auto & obs_bin = gradient_cells_[0];
  obs_bin.reserve(200);
  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      int index = static_cast<int>(master_grid.getIndex(i, j));
      unsigned char cost = master_array[index];
      if (cost == nav2_costmap_2d::FREE_SPACE ) {   //|| (inflate_around_unknown_ && cost == NO_INFORMATION)
        obs_bin.emplace_back(i, j, i, j);
      }
    }
  }

  // Process cells by increasing distance; new cells are appended to the
  // corresponding distance bin, so they
  // can overtake previously inserted but farther away cells
  for (auto & dist_bin : gradient_cells_) {
    dist_bin.reserve(200);
    for (std::size_t i = 0; i < dist_bin.size(); ++i) {
      // Do not use iterator or for-range based loops to
      // iterate though dist_bin, since it's size might
      // change when a new cell is enqueued, invalidating all iterators
      const CellData & cell = dist_bin[i];
      unsigned int mx = cell.x_;
      unsigned int my = cell.y_;
      unsigned int sx = cell.src_x_;
      unsigned int sy = cell.src_y_;
      unsigned int index = master_grid.getIndex(mx, my);

      // ignore if already visited
      if (seen_[index]) {
        continue;
      }

      seen_[index] = true;

      // assign the cost associated with the distance from an obstacle to the cell
      unsigned char cost = nav2_costmap_2d::LETHAL_OBSTACLE;   //costLookup(mx, my, sx, sy)
      unsigned char old_cost = master_array[index];

      double wx, wy, temp_min_wx_, temp_min_wy_, temp_max_wx_, temp_max_wy_;
      float wz=reference_z_map_[index];
      unsigned int temp_min_index, temp_max_index, temp_min_mx_, temp_min_my_, temp_max_mx_, temp_max_my_;
      master_grid.mapToWorld(mx, my, wx, wy);
      float min_z = wz;
      float max_z = wz;
      // Analisi del volume attorno al punto
      for (float dx = -0.2; dx <= 0.2; dx += 0.05) {
        for (float dy = -0.2; dy <= 0.2; dy += 0.05) {
          for (float dz = -0.3; dz <= 0.3; dz += 0.05) {
            octomap::point3d query(wx + dx, wy + dy, wz + dz);
            octomap::OcTreeNode* node = octree_->search(query);
            if (node && octree_->isNodeOccupied(node)) {
              float z = query.z();
              if (z < min_z){
                min_z = z;
                temp_min_wx_ = query.x();
                temp_min_wy_ = query.y();
              } 
              if (z > max_z) {
                max_z = z; 
                temp_max_wx_ = query.x();
                temp_max_wy_ = query.y();
              }
            }
          }
        }
      }
      master_grid.worldToMap(temp_min_wx_, temp_min_wy_, temp_min_mx_, temp_min_my_);
      master_grid.worldToMap(temp_max_wx_, temp_max_wy_, temp_max_mx_, temp_max_my_);
      temp_max_index = master_grid.getIndex(temp_max_mx_, temp_max_my_);
      temp_min_index = master_grid.getIndex(temp_min_mx_, temp_min_my_);
      // RCLCPP_INFO(logger_, "INDEX (%i, %i)", temp_max_index, temp_min_index);
      if(!(temp_max_index==index)){
        reference_z_map_[temp_max_index] = max_z;
        // RCLCPP_INFO(logger_, "Max Z: %f", max_z);
      }
      if(!(temp_min_index==index)){
        reference_z_map_[temp_min_index] = min_z;
        // RCLCPP_INFO(logger_, "Min Z: %f", min_z);
        }
      float dz = max_z - min_z;
      float slope_deg = std::atan2(dz, 0.4) * 180.0 / M_PI;
        if (slope_deg > slope_threshold_deg_ && (old_cost == nav2_costmap_2d::FREE_SPACE))
        {
            master_array[index] = nav2_costmap_2d::LETHAL_OBSTACLE;
        }
    }
      // attempt to put the neighbors of the current cell onto the gradient list
      // if (mx > 0) {
      //   enqueue(index - 1, mx - 1, my, sx, sy);
      // }
      // if (my > 0) {
      //   enqueue(index - size_x, mx, my - 1, sx, sy);
      // }
      // if (mx < size_x - 1) {
      //   enqueue(index + 1, mx + 1, my, sx, sy);
      // }
      // if (my < size_y - 1) {
      //   enqueue(index + size_x, mx, my + 1, sx, sy);
      // }
    // This level of gradient_cells_ is not needed anymore. We can free the memory
    // Note that dist_bin.clear() is not enough, because it won't free the memory
    dist_bin = std::vector<CellData>();
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
    if (distance > cell_gradient_radius_) {
      return;
    }

    const unsigned int r = cell_gradient_radius_ + 2;

    // push the cell data onto the gradient list and mark
    const auto dist = distance_matrix_[mx - src_x + r][my - src_y + r];
    gradient_cells_[dist].emplace_back(mx, my, src_x, src_y);
  }
}

void
GradientLayer::computeCaches()
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());
  if (cell_gradient_radius_ == 0) {
    return;
  }

  cache_length_ = cell_gradient_radius_ + 2;

  // based on the gradient radius... compute distance and cost caches
  if (cell_gradient_radius_ != cached_cell_gradient_radius_) {
    cached_costs_.resize(cache_length_ * cache_length_);
    cached_distances_.resize(cache_length_ * cache_length_);

    for (unsigned int i = 0; i < cache_length_; ++i) {
      for (unsigned int j = 0; j < cache_length_; ++j) {
        cached_distances_[i * cache_length_ + j] = hypot(i, j);
      }
    }

    cached_cell_gradient_radius_ = cell_gradient_radius_;
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
  const int r = cell_gradient_radius_ + 2;    // Gradient radius + 2, used to ensure the entire area is covered.
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


}  // namespace nav2_costmap_2d

PLUGINLIB_EXPORT_CLASS(costmap_plugin::GradientLayer, nav2_costmap_2d::Layer)