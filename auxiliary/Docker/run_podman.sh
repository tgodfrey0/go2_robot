#!/bin/bash

ROS_DISTRO=$1
shift

CONTAINER_NAME="ros2_${ROS_DISTRO}_go2"

xhost +local:root
XAUTH=$HOME/.Xauthority
touch $XAUTH

if ! podman container exists $CONTAINER_NAME; then
    echo "Container does not exist! Creating..."
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
else
    if ! podman container inspect -f '{{.State.Running}}' $CONTAINER_NAME | grep -q "true"; then
        echo "Container is not running! Starting..."
        podman start -i $CONTAINER_NAME
    else
        echo "Attaching to running container..."
        podman attach $CONTAINER_NAME
    fi
fi
