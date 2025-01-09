#!/bin/bash
echo "This image was build for the interface ${INTERFACE}"
source /opt/ros/${ROS_DISTRO}/setup.bash
source /cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="${INTERFACE}" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>'
source /go2_ws/install/setup.bash
