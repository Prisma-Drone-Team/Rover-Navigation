FROM ros:humble

#Uncomment the following line if you get the "release file is not valid yet" error during apt-get
#	(solution from: https://stackoverflow.com/questions/63526272/release-file-is-not-valid-yet-docker)
#RUN echo "Acquire::Check-Valid-Until \"false\";\nAcquire::Check-Date \"false\";" | cat > /etc/apt/apt.conf.d/10no--check-valid-until

RUN echo 'debconf debconf/frontend select Noninteractive' | sudo debconf-set-selections
#Install essential
RUN apt-get update && apt-get install -y

##You may add additional apt-get here
RUN apt-get install dialog apt-utils -y
RUN apt install ros-humble-rviz2 -y
RUN apt install ros-humble-rqt* -y
RUN apt install ros-humble-navigation2 -y
RUN apt install ros-humble-nav2-bringup -y
RUN apt install ros-humble-slam-toolbox -y
RUN apt install ros-humble-turtlebot3-gazebo -y
RUN apt install minicom -y
RUN apt install screen -y
RUN apt install ros-humble-xacro -y
RUN apt install ros-humble-librealsense2* -y
RUN apt install ros-humble-realsense2-* -y
RUN apt install ros-humble-rmw-cyclonedds-cpp -y
RUN apt install ros-humble-joint-state-publisher-gui -y
RUN apt install ros-humble-librealsense2* -y
RUN apt install ros-humble-realsense2-* -y




#Environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV DISPLAY=:0
ENV HOME /home/user
ENV ROS_DISTRO=humble
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

#Add non root user using UID and GID passed as argument
ARG USER_ID
ARG GROUP_ID
RUN addgroup --gid $GROUP_ID user
RUN adduser --disabled-password --gecos '' --uid $USER_ID --gid $GROUP_ID user
RUN echo "user:user" | chpasswd
RUN echo "user ALL=(ALL:ALL) ALL" >> /etc/sudoers

RUN sudo adduser user dialout
RUN sudo adduser user video

USER user

#ROS2 workspace creation and compilation
RUN mkdir -p ${HOME}/ros2_ws/src
WORKDIR ${HOME}/ros2_ws
COPY --chown=user ./src ${HOME}/ros2_ws/src
SHELL ["/bin/bash", "-c"] 
RUN source /opt/ros/${ROS_DISTRO}/setup.bash; rosdep update; rosdep install -i --from-path src --rosdistro humble -y; colcon build --symlink-install

#Add script source to .bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash;" >>  ${HOME}/.bashrc
RUN echo "source ${HOME}/ros2_ws/install/local_setup.bash;" >>  ${HOME}/.bashrc


#Clean image
USER root
RUN rm -rf /var/lib/apt/lists/*
USER user