#!/bin/bash

# Check if ROS_DISTRO argument is provided
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <ROS_DISTRO>"
    exit 1
fi

ROS_DISTRO=$1

# Remove the container associated with the ROS distribution, if it exists
podman rm -f ros2_${ROS_DISTRO}_go2 2>/dev/null

# Remove the image associated with the ROS distribution, if it exists
podman rmi -f localhost/ros2:${ROS_DISTRO}-go2 2>/dev/null

# Clean up dangling images (images not tagged and not referenced by any container)
dangling_images=$(podman images -f "dangling=true" -q)
if [ ! -z "$dangling_images" ]; then
    podman rmi -f $dangling_images  # Force remove dangling images
fi
