cmake_minimum_required(VERSION 3.8)
project(rover_manager)

set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === Dependencies ===

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)

set(dep_pkgs
    rclcpp
    rclcpp_action
    std_msgs
    geometry_msgs
    nav2_msgs
    tf2
    tf2_ros
    pluginlib
)

include_directories(include)

# === Primitives shared library (loaded via pluginlib) ===

add_library(rover_manager_primitives SHARED
            src/goto_primitive.cpp
            src/return_to_base_primitive.cpp
            src/cover_area_primitive.cpp)

ament_target_dependencies(rover_manager_primitives ${dep_pkgs})

install(TARGETS rover_manager_primitives
        DESTINATION lib)

# === RoverManager executable ===

add_executable(rover_manager src/rover_manager.cpp)
ament_target_dependencies(rover_manager ${dep_pkgs})

install(TARGETS rover_manager
        DESTINATION lib/${PROJECT_NAME})

# === Install launch and config ===

install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}/)

# === Plugin exports ===

pluginlib_export_plugin_description_file(rover_manager primitives_plugins.xml)

ament_package()
