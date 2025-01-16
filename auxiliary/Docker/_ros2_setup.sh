#!/bin/bash
echo "This image was build for the interface {CYCLONEDDS_INTERFACE}"
source /opt/ros/${ROS_DISTRO}/setup.bash # This gets sourced automatically somewhere else
source /cyclonedds_ws/install/setup.bash # This causes the hanging
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="{CYCLONEDDS_INTERFACE}" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>'
source /go2_ws/install/setup.bash
echo "Sourcing of go2_ws is DISABLED"
