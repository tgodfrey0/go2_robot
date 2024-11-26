#!/bin/bash
echo $1
docker build --build-arg INTERFACE=$1 -t ros2:foxy-go2 -f Dockerfile . 