#!/bin/bash

if [ -z "$1" ] || [ -z "$2" ]; then
    echo "Uso: $0 <immagine_docker> <nome_container>"
    exit 1
fi

xhost +local:root

WORKSPACE_DIR="$(pwd)/src"
GPU_FLAGS=""

# Se è presente nvidia-smi sul computer, abilitiamo la GPU
if command -v nvidia-smi &> /dev/null && nvidia-smi &> /dev/null; then
    GPU_FLAGS="--gpus all --env=NVIDIA_VISIBLE_DEVICES=all --env=NVIDIA_DRIVER_CAPABILITIES=all"
fi

docker run $GPU_FLAGS \
    --rm -it \
    --name="$2" \
    --net=host \
    --ipc=host \
    --env="DISPLAY=$DISPLAY" \
    --env="USER=user" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:ro" \
    --privileged \
    -v /dev:/dev \
    --volume="${WORKSPACE_DIR}/pkg:/home/user/ros2_ws/src/pkg" \
    "$1"

xhost -local:root