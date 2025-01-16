#!/bin/bash

# Check if the correct number of arguments is provided
if [ "$#" -ne 2 ]; then
    echo "Usage: ./build.sh <IFACE> <DISTRO>"
    exit 1
fi

IFACE=$1
ROS_DISTRO=$2

cp ../_*.sh .
cp ../tmux.conf .

# Execute the build command with appropriate arguments
docker build \
    --squash \
    --build-arg INTERFACE=${IFACE} \
    --build-arg ROS_DISTRO=${ROS_DISTRO} \
    --env DISPLAY=$DISPLAY \
    -t ros2:${ROS_DISTRO}-go2 \
    -f Dockerfile .

rm ./_*.sh
rm ./tmux.conf

echo "Build completed for ROS2 ${ROS_DISTRO} with interface ${IFACE}"
