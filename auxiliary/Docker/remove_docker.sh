#!/bin/bash

ROS_DISTRO=$1
shift

# Remove the container associated with the ROS distribution, if it exists
docker rm -f ros2_${ROS_DISTRO}_go2 2>/dev/null

# Remove the image associated with the ROS distribution, if it exists
docker rmi -f localhost/ros2:${ROS_DISTRO}-go2 2>/dev/null

# Clean up dangling images (images not tagged and not referenced by any container)
dangling_images=$(docker images -f "dangling=true" -q)
if [ ! -z "$dangling_images" ]; then
    docker rmi -f $dangling_images  # Force remove dangling images
fi
