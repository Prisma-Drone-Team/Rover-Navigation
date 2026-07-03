#PER ROBOT HARDWARE:
cd ~
mv org_repo org_repo_SIM        
mv org_repo_REALE org_repo
docker rm -f rover_container
cd ~/org_repo
./docker_run.sh rover_lunar rover_container
colcon build
source install/setup.bash    

#LANCIO FRA2MO

#POI:
docker exec -it rover_container bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# terminale A
ros2 launch gz_migration leo1_rtabmap.launch.py

# terminale B
ros2 launch gz_migration nav_bringup.launch.py namespace:=/robot1

# terminale C
ros2 launch rover_manager manager_robot1.launch.py