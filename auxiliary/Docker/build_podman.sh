#!/bin/bash

IFACE=$1
shift
ROS_DISTRO=$1
shift

# Execute the build command with appropriate arguments
if [ -z "$3" ]; then
  podman build --format docker --squash --build-arg INTERFACE=${IFACE} --build-arg ROS_DISTRO=${ROS_DISTRO} --env DISPLAY=$DISPLAY -t ros2:${ROS_DISTRO}-go2 -f Dockerfile .
else
  echo "Usage: ./build.sh <IFACE> <DISTRO>"
fi
