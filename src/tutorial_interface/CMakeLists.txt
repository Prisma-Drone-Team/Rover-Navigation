cmake_minimum_required(VERSION 3.8)
project(tutorial_interface)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(tutorial_interface REQUIRED)        # CHANGE
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  "msg/Sphere.msg"
  "srv/AddThreeInts.srv"
  DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)


#add_executable(server src/add_two_ints_server.cpp)
#ament_target_dependencies(server
#  rclcpp tutorial_interface)                      #CHANGE

#add_executable(client src/add_two_ints_client.cpp)
#ament_target_dependencies(client
#  rclcpp tutorial_interface)                      #CHANGE

#install(TARGETS
#  server
#  client
#  DESTINATION lib/${PROJECT_NAME})

ament_package()
