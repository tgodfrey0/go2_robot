#!/bin/bash

if [ -z "$2" ]; then
    docker build  --build-arg INTERFACE=$1 -t ros2:foxy-go2 -f Dockerfile .
else
    docker build --build-arg INTERFACE=$1 --build-arg ROS_DISTRO=$2 -t ros2:foxy-go2 -f Dockerfile .
fi
