#!/bin/bash

CONTAINER_NAME="ros2_foxy_go2"

xhost +local:root
XAUTH=$HOME/.Xauthority
touch $XAUTH

if [ ! "$(docker ps -aq -f name=$CONTAINER_NAME)" ]; then
    echo "Container does not exist! Creating..."
    docker run -it \
               --name $CONTAINER_NAME \
               -v /dev:/dev \
               -v /home/$(whoami)/:/home/$(whoami)/ \
               -v /tmp/.X11-unix:/tmp/.X11-unix \
               -v $HOME/.Xauthority:/root/.Xauthority \
               --network="host" \
               --privileged \
               -w /home/$(whoami) \
               ros2:foxy-go2
else
    if [ ! "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
        echo "Container is not running! Starting..."
        docker start -i $CONTAINER_NAME
    fi
    echo "Attaching to running container..."
    docker attach $CONTAINER_NAME
fi
