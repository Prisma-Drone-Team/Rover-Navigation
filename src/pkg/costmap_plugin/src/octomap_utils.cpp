#include "octomap_utils.hpp"

#include <cmath>

namespace gradient_utils
{

std::unique_ptr<octomap::OcTree> filterOctomapWithinRadius(
  const std::shared_ptr<octomap::ColorOcTree> & octree,
  const nav2_costmap_2d::Costmap2D & costmap,
  std::vector<float> & reference_z_map, std::vector<bool> & occupied,
  double robot_x, double robot_y, double robot_z,
  double radius, double & min_dist)
{
  double resolution = octree->getResolution();  
  auto logger = rclcpp::get_logger("gradient_utils");
  new_time::Clock clk;
  auto filtered_tree = std::make_unique<octomap::OcTree>(octree->getResolution());
  octomap::point3d bbx_min(robot_x - radius, robot_y - radius, robot_z - radius);
  octomap::point3d bbx_max(robot_x + radius, robot_y + radius, robot_z + radius);

  // Conversione in chiavi per il bounding box
  octomap::OcTreeKey min_key, max_key;
  if (!octree->coordToKeyChecked(bbx_min, min_key) || !octree->coordToKeyChecked(bbx_max, max_key)) {
    RCLCPP_WARN(logger, "Bounding box fuori dai limiti dell'OctoMap.");
    return filtered_tree;
  }

  for (auto it = octree->begin_leafs_bbx(min_key, max_key); it != octree->end_leafs_bbx(); ++it) { //Controllo octomap filtrata
  // for (auto it = octree->begin_leafs(); it != octree->end_leafs(); ++it) {  //Controllo dell'interca octomap
    if (octree->isNodeOccupied(*it)) {
      double wx = it.getX(), wy = it.getY(), wz = it.getZ();
      double dist = std::sqrt((wx - robot_x)*(wx - robot_x) + (wy - robot_y)*(wy - robot_y) + (wz - robot_z)*(wz - robot_z));

      unsigned int mx, my;
      if (costmap.worldToMap(wx, wy, mx, my)) {
        unsigned int idx = costmap.getIndex(mx, my);
        occupied[idx] = true;
        if (idx <= reference_z_map.size() && std::abs(wz) >= reference_z_map[idx])  {
          // if ((wz >= -resolution && wz <= resolution)) {
          //   reference_z_map[idx] = 0;
          // } else {
            reference_z_map[idx] = wz;
          // }
        }
      }

      // if (dist <= radius && (wz < -0.1 || wz > 0.1)) {
      //   filtered_tree->updateNode(octomap::point3d(wx, wy, wz), true);
      //   if(dist <= min_dist){
      //     min_dist = dist;
      //   }
      // }
    }
  }
  filtered_tree->updateInnerOccupancy();
  double elapsed_sec = clk.toc();  // ferma il cronometro e misura

  // RCLCPP_INFO(logger, "OctoMap filtering took %.4f seconds", elapsed_sec);
  
  return filtered_tree;
}

bool collisionCheck(
  const std::shared_ptr<octomap::ColorOcTree> & octree,
  double robot_x, double robot_y, double robot_z,
  double radius)
{
  auto logger = rclcpp::get_logger("gradient_utils");
  bool collision = false;
  double resolution = octree->getResolution();
  new_time::Clock clk;

  octomap::point3d bbx_min(robot_x - radius, robot_y - radius, robot_z - radius);
  octomap::point3d bbx_max(robot_x + radius, robot_y + radius, robot_z + radius);

  // Conversione in chiavi per il bounding box
  octomap::OcTreeKey min_key, max_key;
  if (!octree->coordToKeyChecked(bbx_min, min_key) || !octree->coordToKeyChecked(bbx_max, max_key)) {
    RCLCPP_WARN(logger, "Bounding box fuori dai limiti dell'OctoMap.");
    return false;
  }

  for (auto it = octree->begin_leafs_bbx(min_key, max_key); it != octree->end_leafs_bbx(); ++it) {
    if (octree->isNodeOccupied(*it)) {
      double  wz = it.getZ(); // wx = it.getX(), wy = it.getY(),
      // double dist = (wx - robot_x)*(wx - robot_x) + (wy - robot_y)*(wy - robot_y) + (wz - robot_z)*(wz - robot_z);

      if ((wz <= -resolution || wz >= resolution)) { //dist <= radius*radius && 
        collision = true;
        break;
      }
    }
  }  
  double elapsed_sec = clk.toc();  // ferma il cronometro e misura
  RCLCPP_INFO(logger, "OctoMap filtering took %.4f seconds", elapsed_sec);
  
  return collision;
}


// double getMinDist()
// {
// auto logger = rclcpp::get_logger("gradient_utils");
//   RCLCPP_INFO(logger, "Nearest obstacle distance: %f", min_dist);
//   return min_dist;
// }

}  // namespace gradient_utils