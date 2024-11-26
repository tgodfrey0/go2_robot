# Running with Docker

To make it easier to get started with this SDK on a Go2 robot, a comprehensive Dockerile was created. The container supports window forwarding allowing for the use of GUI-based tools like `rviz2`.

To build the Docker container, run

```bash
./build.sh INTERFACE
```

where `INTERFACE` is the network interface to which the robot is connected. For example, if the Go2 was connected over ethernet and the interface was `eth0`, one would run `./build eth0`.

To run the container, simply run

```bash
./run.sh
```

You will now be in the container and should be able to see the topics from the robot. This can be verified by running `ros2 topic list`.

The SDK can now be run and the robot can be used.

```bash
ros2 launch go2_bringup go2.launch.py
```

If using the Docker container, it is easy to change the selected ROS2 distribution. When building the container, simply include the name of the distribution after the interface name, e.g. `./build.sh eth0 humble`.

# Running without Docker

It is also easy to run this without using a Docker container. We need to install `cyclonedds 0.10` and configure it with the correct interface. 

**Ensure that ROS2 has not been sourced yet.**

First, we need to gather the sources.

```bash
cd ~/
git clone https://github.com/unitreerobotics/unitree_ros2
mv unitree_ros2/cyclonedds_ws ~/
rm -rf ~/unitree_ros2
cd ~/cyclonedds_ws/src
git clone https://github.com/ros2/rmw_cyclonedds -b foxy
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x 
```

We can then build the `cyclonedds` package.

```bash
cd ~/cyclonedds_ws
colcon build --packages-select cyclonedds
source /opt/ros/foxy/setup.bash
colcon build
```

Now we can create a setup file which we can use to source the ROS2 installation and set the correct environment variables.

Create the file `~/ros2_rmw_setup.sh` with the following content. Replace `INTERFACE` with the correct network interface.

```bash
#!/bin/bash
echo "Setup ros2 for use with Go2"
source /opt/ros/foxy/setup.bash
source /cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="INTERFACE" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>'
```

To use this script, simply run `source ~/ros2_rmw_setup.sh`.

After this, the correct version of CycloneDDS should be running and configured, and you should be communicating with the Go2 ROS2 instance. Again, this can be verified using `ros2 topic list`. 

_This document was written by Toby Godfrey (t.godfrey \~at\~ soton.ac.uk)_
