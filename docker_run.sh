xhost +local:root
IMAGE_ID=$(docker images -q $1)
#docker run --rm -it --name=$2 --net=host --env="DISPLAY=$DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:ro" --volume=/dev/ttyACM0 --volume=/src:/home/drone/Tesi-Rover/src $1
docker run --rm -it --name=$2 --net=host --env="DISPLAY=$DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:ro" --privileged -v /dev:/dev --volume=/home/Tesi-Rover/src:/home/stefano/ros2_ws/src $1
xhost -local:root
