## Overview
This workspace is a ROS2 porting of an indoor navigation project, now evolved into a **multi-rover** outdoor/lunar exploration system. It implements full integration with SLAM and the Navigation2 ROS2 package.

This repository is exclusively built for ROS2. It is being tested on Ubuntu 20.04/22.04 with ROS2-Humble.

## Table of Contents
1. [Workspace Architecture](#workspace-architecture)
2. [Prerequisites](#prerequisites)
3. [Packages used in Dockerfile](#packages-used-in-dockerfile)
4. [Image Compilation and Execution](#image-compilation-and-execution)
5. [Usage and Launch Instructions](#usage-and-launch-instructions)
6. [Configuration and Parameters](#configuration-and-parameters)

## Workspace Architecture
The workspace has evolved through three main phases, and the packages reflect this history:

### 1. Original Simulation (Single Rover - Gazebo)
- `rover_gazebo`: Responsible for initializing the Ignition Gazebo environment, loading the lunar world, and spawning the single rover.
- `rover_description_pkg`: Holds the URDF/xacro descriptions and meshes for the original single-rover simulation.
- `rover_bringup`: Contains the core configuration and launch files for bringing up SLAM (RTAB-Map/Slam Toolbox), Navigation2, and Robot Localization for a single rover.

### 2. Multi-rover (Gazebo)
- `gz_migration`: Introduces new multi-rover capabilities. Contains updated rover models and the complex launch ecosystem required to spawn and manage two concurrent rovers in Gazebo.
- `costmap_plugin`: A custom ROS2 plugin used to enhance SLAM/Nav capabilities, shared by both Gazebo and Webots simulations.
- `custom_explorer`: A task-assignment package that dispatches exploration goals to the rovers. *(Note: Compatible with Webots only if the spawned rovers are explicitly named `robot1` and `robot2`).*
- `rover_manager`: A high-level supervisor node that manages goals and dynamic TF frames. It supports both Gazebo (using dedicated launch files for each rover) and Webots (using a single parameterized launch file).
- `leo_gz_plugins`: Contains custom Gazebo plugins required for the proper simulation of the Leo Rover's hardware and sensors in the Ignition/Gazebo environment.

### 3. Multi-rover (Webots)
- `rover_webots`: Provides the multi-rover simulation environment using Webots. It introduces a flexible, parameter-based architecture: each dedicated launch file (for spawning, SLAM, or Navigation) can configure any rover simply by passing the robot's name as an argument.

## Prerequisites
Before setting up the project, you have to install docker. If you already installed docker, go to the next session
```sh
sudo apt-get update
sudo apt-get install ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

# Install the Docker packages
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Add docker user
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
```
Then log out and log in.

## Packages used in Dockerfile
The provided `Dockerfile` installs all necessary dependencies for the project, including:
- **Simulators**: Webots (`webots`, `ros-humble-webots-ros2`) and Ignition Gazebo (`ignition-fortress`)
- **Navigation & SLAM**: `navigation2`, `nav2-bringup`, `slam-toolbox`, `rtabmap-ros`, `robot-localization`
- **Teleoperation**: Joystick drivers and teleop packages (`joy`, `teleop-twist-joy`)
- **Sensors & Utilities**: RealSense dependencies (`librealsense2`, `realsense2`), `joint-state-publisher-gui`
- **Middleware**: CycloneDDS (`rmw-cyclonedds-cpp`)

Additionally, the following external repositories are compiled from source inside the workspace:
- [TEB Local Planner](https://github.com/rst-tu-dortmund/teb_local_planner)
- [Costmap Converter](https://github.com/rst-tu-dortmund/costmap_converter)

## Image Compilation and Execution

1. Clone the repository:
```sh
git clone https://github.com/Prisma-Drone-Team/Rover-Navigation.git
cd Rover-Navigation
```
2. Open a terminal in the repo and execute the build script:
```sh
./docker_build.sh <IMAGE_NAME>
```
where `<IMAGE_NAME>` is a name for the image you want to build.

3. In the terminal in the repo, execute the run script:
```sh
./docker_run.sh <IMAGE_NAME_OR_ID> <CUSTOM_CONTAINER_NAME>
```
where `<IMAGE_NAME_OR_ID>` is the name (or the ID) of the image you want to run, and `<CUSTOM_CONTAINER_NAME>` is a custom name you choose to assign to the container.

### Notes 
1. The container's root password is "user" by default.

2. The container will be automatically destroyed once exited (`--rm` flag). If you want to attach additional terminals to a running container, you can use the provided script:
```sh
./docker_attach.sh <CONTAINER_NAME>
```
3. It is recommended to check the correct time for successful image creation.
4. All `apt-get` performed inside the container will be removed once the container is closed. Please add all new dependencies to the Dockerfile and rebuild the image.

## Usage and Launch Instructions

### Setup Environment
1. Create a container using this project image on your local machine.
   *(Note: If you are using a distributed setup with a separate controller PC, ensure both machines share the same `ROS_DOMAIN_ID` and their system times are synchronized).*
2. Inside the container terminal, build the workspace and source it:
```sh
colcon build --symlink-install
source ./install/setup.bash
```

### Launching the Simulation

Remember to source the workspace (`source ./install/setup.bash`) in every new terminal you open.

#### Option A: Webots (Recommended)
Webots uses a fully parameterized architecture. You can spawn and manage multiple rovers by specifying a `<ROVER_NAME>`. Open a new terminal for each command:

1. **Lunar World:**
   ```sh
   ros2 launch rover_webots moon.launch.py
   ```
2. **Spawn Rover:**
   ```sh
   ros2 launch rover_webots leo.launch.py name:=<ROVER_NAME>
   ```
3. **SLAM (RTAB-Map):**
   ```sh
   ros2 launch rover_webots rtabmap.launch.py name:=<ROVER_NAME>
   ```
4. **Navigation2:**
   ```sh
   ros2 launch rover_webots navigation.launch.py name:=<ROVER_NAME>
   ```
5. **Task Manager:**
   ```sh
   ros2 launch rover_manager manager.launch.py name:=<ROVER_NAME>
   ```

#### Option B: Gazebo (Multi-rover)
The Gazebo multi-rover setup uses dedicated launch files for `robot1` and `robot2`.

1. **Lunar World:**
   ```sh
   ros2 launch gz_migration 2_robots_world.launch.py
   ```
2. **Spawn Rovers:**
   ```sh
   ros2 launch gz_migration 1_leo_rtab.launch.py
   ros2 launch gz_migration 2_leo_rtab.launch.py
   ```
3. **SLAM (RTAB-Map):**
   ```sh
   ros2 launch gz_migration leo1_rtabmap.launch.py
   ros2 launch gz_migration leo2_rtabmap.launch.py
   ```
4. **Navigation2:**
   ```sh
   ros2 launch gz_migration namespace_1.launch.py
   ros2 launch gz_migration namespace_2.launch.py
   ```
5. **Task Manager:**
   ```sh
   ros2 launch rover_manager manager_robot1.launch.py
   ros2 launch rover_manager manager_robot2.launch.py
   ```

### Additional Features & Notes
- **Legacy Utilities:** Launch files for the original single-rover setup, teleop, and rampa are still available in their respective packages but omitted here for brevity.
- **Custom Explorer:** The `custom_explorer` package provides standalone task-assignment executables (e.g., `lunar_explorer`) that can be launched via `ros2 run custom_explorer <NODE_NAME>`.
- **Manager Commands:** You can manually trigger `rover_manager` primitives (e.g., `goto`, `cover_area`, `return_to_base`) by publishing to the `/<ROVER_NAME>/seed_pdt_rover/command` topic.

### Visualization & Monitoring

**1. RViz2 (Navigation & Maps)**
To visualize the rover, the generated map, and the costmaps, you can use the pre-configured RViz2 layout. This interface also allows you to manually send navigation goals using the "2D Goal Pose" tool.
```sh
rviz2 -d src/pkg/rover_webots/rover.rviz
```
*(Note: The provided `.rviz` configuration file is currently hardcoded to subscribe to topics and frames for a rover named `leo1`. If you spawn a rover with a different name, you will need to adjust the topic namespaces manually inside RViz).*

**2. Camera Feeds**
To view the live feeds from the rovers' cameras:
```sh
ros2 run rqt_image_view rqt_image_view
```

**3. Node Graph**
To visualize the active ROS2 nodes and their communication topics (highly recommended for debugging multi-rover setups):
```sh
rqt_graph
```

## Configuration and Parameters

The repository relies on several YAML configuration files to tune the navigation stack, task manager, and SLAM parameters. Thanks to the parameterized architecture in Webots, many of these files use dynamic placeholders (e.g., `<robot_name>`) that are resolved automatically at launch.

### 1. Navigation2 Parameters
Nav2 is tuned using parameter files that define the behavior of the Costmaps, Planners (e.g., TEB Local Planner), and Controllers.
- **Webots**: `src/pkg/rover_webots/params/navigation_params.yaml`
- **Gazebo (Multi-rover)**: `src/pkg/gz_migration/params/leo1_navigation_params.yaml` and `leo2_navigation_params.yaml`
- **Original Single-rover**: `src/pkg/rover_bringup/params/nav2_sim_params.yaml`

*(Note: Custom plugins like `costmap_plugin` are also configured within these files).*

### 2. Task Manager (`rover_manager`)
The supervisor node relies on parameter files to configure its action primitives (`goto`, `cover_area`, `return_to_base`) and target topics.
- **Webots**: `src/pkg/rover_manager/config/manager_params.yaml`
- **Gazebo**: `src/pkg/rover_manager/config/robot1_manager_params.yaml` and `robot2_manager_params.yaml`

### 3. SLAM & Localization
Settings for RTAB-Map and Robot Localization (EKF) are primarily located in:
- `src/pkg/rover_bringup/config/localization_ekf.yaml`
- `src/pkg/rover_bringup/config/slam_sim_params_online.yaml`

*(Note: In the Webots and Gazebo multi-rover setups, RTAB-Map parameters are injected dynamically via launch arguments inside their respective Python launch files rather than using static YAML files).*

## Developer Notes

### Odometry Implementation
In both Gazebo and Webots, odometry is currently calculated using the **absolute coordinates** from the simulator rather than from wheel encoder movements (which would require additional sensor noise tuning). 
- In Webots, this is achieved using the `absolute_odom_plugin` script.
- **Future Work:** If you wish to re-implement differential drive odometry (wheel kinematics) in Webots, you must replace `absolute_odom_plugin` with `diff_drive_plugin.py` inside `leo_webots.urdf`, and set the `supervisor` field to `FALSE` in the `Rover.proto` file.