#pragma once

#include <memory>
#include <octomap/OcTree.h>
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "rclcpp/rclcpp.hpp"
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include "seed_time.h"


namespace gradient_utils
{

std::unique_ptr<octomap::OcTree> filterOctomapWithinRadius(
  const std::shared_ptr<octomap::ColorOcTree> & octree,
  const nav2_costmap_2d::Costmap2D & costmap,
  std::vector<float> & reference_z_map, std::vector<bool> & occupied,
  double robot_x, double robot_y, double robot_z,
  double radius, double & min_dist);

bool collisionCheck(
  const std::shared_ptr<octomap::ColorOcTree> & octree,
  double robot_x, double robot_y, double robot_z,
  double radius);

//   double getMinDist();

}
