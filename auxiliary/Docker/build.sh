#!/bin/bash

# Function to check if podman is available and being used as docker
is_podman() {
    if command -v docker &> /dev/null; then
        if docker --version | grep -qi "podman"; then
            return 0  # True, it's podman
        else
            return 1  # False, it's docker
        fi
    elif command -v podman &> /dev/null; then
        return 0  # True, it's podman
    else
        echo "Neither Docker nor Podman is installed."
        exit 1
    fi
}

# Set the build command based on whether it's Docker or Podman
if is_podman; then
    build_command="podman build --format docker"
else
    build_command="docker build"
fi

IFACE=$1
shift
ROS_DISTRO=$1
shift

# Execute the build command with appropriate arguments
if [ -z "$3" ]; then
  $build_command --squash --build-arg INTERFACE=${IFACE} --build-arg ROS_DISTRO=${ROS_DISTRO} --env DISPLAY=$DISPLAY -t ros2:${ROS_DISTRO}-go2 -f Dockerfile .
else
  echo "Usage: ./build.sh <IFACE> <DISTRO>"
fi
