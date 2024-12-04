#!/bin/bash

xhost +local:root
docker run  -it \
            -v /dev:/dev \
            -v /home/$(whoami)/:/home/$(whoami)/ \
            -v /tmp/.X11-unix:/tmp/.X11-unix \
            --network="host" \
            --privileged \
            -w /home/$(whoami) \
            ros2:foxy-go2

