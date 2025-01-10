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

# Execute the build command with appropriate arguments
if [ -z "$2" ]; then
    $build_command --build-arg INTERFACE=$1 -t ros2:foxy-go2 -f Dockerfile .
else
    $build_command --build-arg INTERFACE=$1 --build-arg ROS_DISTRO=$2 -t ros2:foxy-go2 -f Dockerfile .
fi
