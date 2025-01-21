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

# Remove the container associated with the ROS distribution, if it exists
$CMD rm -f $CONTAINER_NAME 2>/dev/null

# Remove the image associated with the ROS distribution, if it exists
$CMD rmi -f localhost/$IMAGE_NAME 2>/dev/null

# Clean up dangling images (images not tagged and not referenced by any container)
dangling_images=$($CMD images -f "dangling=true" -q)
if [ ! -z "$dangling_images" ]; then
    $CMD rmi -f $dangling_images  # Force remove dangling images
fi
