ARG ROS_DISTRO=foxy

########################################
# Base image #
########################################
FROM osrf/ros:${ROS_DISTRO}-desktop as base

ARG INTERFACE

########################################
# Environment variables #
########################################
ENV ROS_DISTRO=${ROS_DISTRO}
ENV ROS_VERSION=2
ENV ROS_PYTHON_VERSION=3
ENV ROS_PACKAGE_PATH=/opt/ros/${ROS_DISTRO}/share
ENV DISPLAY="unix:0"

########################################
# Set the shell #
########################################
SHELL ["/bin/bash", "-c"]

########################################
# Install packages #
########################################
RUN apt-get update -y
RUN apt-get upgrade -y
RUN apt-get install -y \
  ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
  ros-${ROS_DISTRO}-realsense2-camera \
  ros-${ROS_DISTRO}-rosidl-generator-dds-idl \
  ros-${ROS_DISTRO}-ament-cmake \
  ros-${ROS_DISTRO}-ament-cmake-ros \
  ros-${ROS_DISTRO}-geometry-msgs \
  ros-${ROS_DISTRO}-std-msgs \
  ros-${ROS_DISTRO}-tf2-ros \
  ros-${ROS_DISTRO}-rosidl-default-generators \
  ros-${ROS_DISTRO}-ament-lint-common \
  ros-${ROS_DISTRO}-rclcpp-action \
  ros-${ROS_DISTRO}-xacro \
  ros-${ROS_DISTRO}-launch-ros \
  ros-${ROS_DISTRO}-pointcloud-to-laserscan \
  ros-${ROS_DISTRO}-rqt-robot-steering \
  libboost-all-dev \
  libyaml-cpp-dev \
  iproute2 \
  iputils-ping \
  nano \
  ninja-build \
  tmux

########################################
# Setup CycloneDDS file #
########################################
WORKDIR /
RUN git clone https://github.com/unitreerobotics/unitree_ros2
RUN mv unitree_ros2/cyclonedds_ws /
RUN rm -rf /unitree_ros2

WORKDIR /cyclonedds_ws/src
RUN git clone https://github.com/ros2/rmw_cyclonedds -b ${ROS_DISTRO}
RUN git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x

WORKDIR /cyclonedds_ws
RUN colcon build --packages-select cyclonedds \
  && source /opt/ros/${ROS_DISTRO}/setup.bash \
  && colcon build

WORKDIR /
COPY _ros2_rmw_setup.sh /ros2_rmw_setup.sh
RUN chmod +x /ros2_rmw_setup.sh
# RUN echo "source /ros2_rmw_setup.sh" >> /root/.bashrc

########################################
# Setup Go2 repos #
########################################
RUN mkdir -p /go2_ws/src
WORKDIR /go2_ws/src
RUN git clone https://github.com/tgodfrey0/go2_robot.git
RUN git clone https://github.com/nlohmann/json.git
RUN git clone --recurse-submodules https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0.git
RUN mv json/include/nlohmann go2_robot/go2_driver/include/
RUN rm -rf json

########################################
# Build packages #
########################################
WORKDIR /go2_ws
RUN . /opt/ros/${ROS_DISTRO}/setup.bash \
  # && . install/setup.bash \
  && rosdep update \
  && rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO} --include-eol-distros \
  && colcon build --symlink-install \
  && . install/setup.bash

########################################
# Copy tmux configuration #
########################################
COPY tmux.conf /root/.tmux.conf

########################################
# Set entrypoint and default command #
########################################
COPY _ros_entrypoint_tmux.sh /ros_entrypoint_tmux.sh
RUN chmod +x /ros_entrypoint_tmux.sh

# Set the entrypoint
CMD ["/ros_entrypoint_tmux.sh"]

# ENTRYPOINT [ "/ros_entrypoint.sh" ]
# CMD [ "bash" ]
