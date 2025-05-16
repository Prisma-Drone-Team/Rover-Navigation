cmake_minimum_required(VERSION 3.8)
project(gz_migration)

set(CMAKE_CXX_STANDARD 14)

# find dependencies
#find_package(nav2_bt_navigator_1 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rtabmap_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
#find_package(ros_gz_interfaces REQUIRED)
#find_package(ignition-transport12 REQUIRED)
#find_package(ignition-msgs REQUIRED)
include_directories(${Protobuf_INCLUDE_DIRS}
                    include)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  )
  
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
  )

install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
  )
 
install(DIRECTORY meshes_2/
  DESTINATION share/${PROJECT_NAME}/meshes_2
  )

install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
  )

install(DIRECTORY models_2/
  DESTINATION share/${PROJECT_NAME}/models_2
  )

install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
  )

install(DIRECTORY scan_mode/
  DESTINATION share/${PROJECT_NAME}/scan_mode
  )
  
install(DIRECTORY params/
  DESTINATION share/${PROJECT_NAME}/params
  )

install(DIRECTORY map/
  DESTINATION share/${PROJECT_NAME}/map
  )

install(DIRECTORY include/
  DESTINATION include/
  )
	
add_executable(robot1_odom_sim src/leo1_odometry.cpp)
ament_target_dependencies(robot1_odom_sim rclcpp tf2 tf2_ros tf2_geometry_msgs tf2_msgs geometry_msgs nav_msgs)
install(TARGETS
	robot1_odom_sim
	DESTINATION lib/${PROJECT_NAME})

add_executable(robot2_odom_sim src/leo2_odometry.cpp)
ament_target_dependencies(robot2_odom_sim rclcpp tf2 tf2_ros tf2_geometry_msgs tf2_msgs geometry_msgs nav_msgs)
install(TARGETS
	robot2_odom_sim
	DESTINATION lib/${PROJECT_NAME})

add_executable(robot1_exploration src/leo1_exploration.cpp)
ament_target_dependencies(robot1_exploration rclcpp geometry_msgs nav2_msgs rclcpp_action tf2 tf2_geometry_msgs tf2_msgs)
install(TARGETS
	robot1_exploration
	DESTINATION lib/${PROJECT_NAME})
	
add_executable(robot2_exploration src/leo2_exploration.cpp)
ament_target_dependencies(robot2_exploration rclcpp geometry_msgs nav2_msgs rclcpp_action tf2 tf2_geometry_msgs tf2_msgs)
install(TARGETS
	robot2_exploration
	DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  #find_package(realsense_gazebo_plugin REQUIRED)
  #find_package(ignition-plugin REQUIRED COMPONENTS register)
  #find_package(ignition-gazebo REQUIRED)
  
#  add_library(
#    realsense_gazebo_plugin SHARED src/pkg/realsense_gazebo_plugin/src/RealSensePlugin.cpp
    #src/pkg/realsense_gazebo_plugin/src/gazebo_ros_realsense.cpp
#  )
#  target_link_libraries(realsense_gazebo_plugin
#    ignition-gazebo6::ignition-gazebo6
#  )	
#  install(
#    TARGETS realsense_gazebo_plugin
#    LIBRARY DESTINATION lib
#  )
  
#  add_library(${PROJECT_NAME} SHARED src/realsense_gazebo_plugin/src/RealSensePlugin.cpp)
#  target_link_libraries(${PROJECT_NAME} PRIVATE ignition-plugin::register ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER})
#  target_include_directories(${PROJECT_NAME} PRIVATE include)
#  target_compile_definitions(${PROJECT_NAME} PRIVATE "IGNITION_VISIBLE=__attribute__((visibility(\"default\")))")

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_sim_resource_path.sh")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_sim_resource_path.dsv")

ament_package()
