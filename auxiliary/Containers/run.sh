#!/bin/bash

is_podman() {
    if command -v podman &> /dev/null; then
        return 0
    elif command -v docker &> /dev/null; then
        return 1
    else
        echo "Neither Podman nor Docker is installed"
        exit 1
    fi
}

if is_podman; then
    CMD="podman"
else
    CMD="docker"
fi

# Check if ROS_DISTRO argument is provided
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <ROS_DISTRO>"
    exit 1
fi

ROS_DISTRO=$1
IMAGE_NAME="ros2:go2-${ROS_DISTRO}"
CONTAINER_NAME="ros2_go2_${ROS_DISTRO}"

# Allow local X11 connections
xhost +local:root
XAUTH=$HOME/.Xauthority
touch $XAUTH

if [ ! "$($CMD ps -aq -f name=$CONTAINER_NAME)" ]; then
    echo "Container does not exist! Creating..."
    $CMD run -it \
        --name $CONTAINER_NAME \
        -v /dev:/dev:Z \
        -v /home/$USER/:/home/$USER/:Z \
        -v /tmp/.X11-unix:/tmp/.X11-unix:Z \
        -v $HOME/.Xauthority:/root/.Xauthority:Z \
        --network="host" \
        --privileged \
        -w $HOME \
        ${IMAGE_NAME}
else
    if [ ! "$($CMD ps -q -f name=$CONTAINER_NAME)" ]; then
        echo "Container is not running! Starting..."
        $CMD start -i $CONTAINER_NAME
    fi
    echo "Attaching to running container..."
    $CMD attach $CONTAINER_NAME
fi
