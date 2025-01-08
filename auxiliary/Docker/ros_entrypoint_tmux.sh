#!/bin/bash
set -e

# Source ROS setup
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# Source your custom setup script
source /ros2_rmw_setup.sh
echo "source /ros2_rmw_setup.sh" >> /root/.bashrc

# Start tmux session with a single window
tmux new-session -d -s ros2_session

# Execute the command passed to the docker run
exec "$@"
