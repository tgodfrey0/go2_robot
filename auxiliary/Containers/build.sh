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
    CMD="podman build --format docker"
else
    CMD="docker build"
fi

# Check if the correct number of arguments is provided
if [ "$#" -ne 2 ]; then
    echo "Usage: ./build.sh <IFACE> <DISTRO>"
    exit 1
fi

IFACE=$1
ROS_DISTRO=$2
IMAGE_NAME="ros2:go2-${ROS_DISTRO}"
CONTAINER_NAME="ros2_go2_${ROS_DISTRO}"

if $CMD --squash \
        --build-arg INTERFACE=${IFACE} \
        --build-arg ROS_DISTRO=${ROS_DISTRO} \
        --env DISPLAY=$DISPLAY \
        -t ${IMAGE_NAME} \
        -f Containerfile .; then

    echo "Build completed for ROS2 ${ROS_DISTRO} with interface ${IFACE}"
else
    echo "Build failed for for ROS2 ${ROS_DISTRO} with interface ${IFACE}"
fi
