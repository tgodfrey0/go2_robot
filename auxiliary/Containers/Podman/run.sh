#!/bin/bash

# Check if ROS_DISTRO argument is provided
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <ROS_DISTRO>"
    exit 1
fi

ROS_DISTRO=$1
CONTAINER_NAME="ros2_${ROS_DISTRO}_go2"

# Allow local X11 connections
xhost +local:root

# Ensure .Xauthority file exists
XAUTH=$HOME/.Xauthority
touch $XAUTH

# Function to run the container
run_container() {
    podman run -it \
        --name $CONTAINER_NAME \
        -v /dev:/dev:Z \
        -v /home/$(whoami)/:/home/$(whoami)/:Z \
        -v /tmp/.X11-unix:/tmp/.X11-unix:Z \
        -v $HOME/.Xauthority:/root/.Xauthority:Z \
        --network="host" \
        --privileged \
        -w /home/$(whoami) \
        --ulimit nofile=1024:65536 \
        ros2:${ROS_DISTRO}-go2
}

# Check if the container exists
if ! podman container exists $CONTAINER_NAME; then
    echo "Container does not exist! Creating..."
    run_container
else
    # Check if the container is running
    if ! podman container inspect -f '{{.State.Running}}' $CONTAINER_NAME | grep -q "true"; then
        echo "Container is not running! Starting..."
        podman start -i $CONTAINER_NAME
    fi
    echo "Attaching to running container..."
    podman attach $CONTAINER_NAME

fi
