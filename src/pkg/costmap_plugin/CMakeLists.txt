cmake_minimum_required(VERSION 3.8)
project(costmap_plugin)
set(CMAKE_CXX_STANDARD 14)

# === Dependencies ===

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)

set(lib_name ${PROJECT_NAME}_core)

set(dep_pkgs
    rclcpp
    nav2_costmap_2d
    pcl_conversions
    pluginlib
    octomap_msgs
    octomap
    sensor_msgs
    )

# === Build ===

add_library(${lib_name} SHARED
            src/gradient_layer.cpp
            src/octomap_utils.cpp
            src/slope_cost_map.cpp
            src/roughness_cost_map.cpp
            src/shadow_cost_map.cpp)

include_directories(include)
include_directories(include/costmap_plugin)
include_directories(octomap)

# === Installation ===

install(TARGETS ${lib_name}
        DESTINATION lib)

target_link_libraries(${lib_name}
        octomap
      )

# === Ament work ===

# Export GradientLayer as a nav2_costmap_2d::Layer plugin
pluginlib_export_plugin_description_file(nav2_costmap_2d gradient_layer.xml)

# Export CostMapBase sub-plugins (SlopeCostMap, RoughnessCostMap, ShadowCostMap, etc.)
pluginlib_export_plugin_description_file(costmap_plugin cost_map_plugins.xml)

ament_target_dependencies(${lib_name} ${dep_pkgs})
ament_package()
