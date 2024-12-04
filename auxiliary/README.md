# Additional Guidance

## Accessing the Go2 over WiFi

If your WiFi interface is not attached through the `eth0` interface (e.g. a USB adapter), additional configuration is needed. 

The internal topics are not published over WiFi. The topics from this package can be published over WiFi.

_This has been tested using a USB WiFi adapter._

### Making a Hotspot

Plug the WiFi adapter into the Go2 and check that it has been recognised by running `ip a`.

We must check that the WiFi adapter can be used as a hotspot by running

```bash
iw list
```

In the output, find the area titled "Supported interface modes" and ensure that `AP` is listed.

```bash
Supported interface modes:
		 * IBSS
		 * managed
		 * AP
		 * AP/VLAN
		 * monitor
		 * P2P-client
		 * P2P-GO
		 * P2P-device
```

Then we can use `nmcli` to setup the hotspot.

```bash
sudo nmcli dev wifi hotspot ifname <INTERFACE NAME> ssid <NETWORK NAME> password "<NETWORK PASSWORD>"
```

An example command would be `sudo nmcli dev wifi hotspot ifname wlan0 ssid go2 password "hotspot0000"`

This will create a WiFi hotspot that other computers can connect to.

To make the hotspot automatically start when the robot is turned on, we must edit the configuration file.

Find the configuration file by running 

```bash
sudo ls /etc/NetworkManager/system-connections/
```

Then open it in an editor. 

```bash
sudo nano /etc/NetworkManager/system-connections/<CONFIG FILE>
```

Then find the line that says `autoconnect=false` and change it to `autoconnect=true`. Now the hotspot will start when the Go2 turns on.

### `CycloneDDS` Configuration

Now that we have a WiFi network other devices can connect to, we need to tell ROS that we want to use that interface.

By default, the `CycloneDDS` configuration on the Go2 only uses `eth0` for ROS.

We must open `~/cyclonedds_ws/cyclonedds.xml`, but it is a good idea to make a backup.

```bash
cp ~/cyclonedds_ws/cyclonedds.xml ~/cyclonedds_ws/cyclonedds.xml.ORIGINAL
```

Then we can open it by running

```bash
nano ~/cyclonedds_ws/cyclonedds.xml
```

Find the line that says

```xml
<NetworkInterface name="eth0" priority="default" multicast="default" />
```

Add a line below for the wireless interface

```xml
<NetworkInterface name="eth0" priority="default" multicast="default" />
<NetworkInterface name="<WIRELESS INTERFACE>" priority="default" multicast="default" />
```

Save and close the file, then run `ros2 daemon stop` and `ros2 daemon start` to ensure the changes take place. Check that the internal nodes and topics are still visible.

### ROS Over WiFi

As the internal MCU topics are not published over WiFi, we must start the `go2_bringup` package by running

```bash
ros2 launch go2_bringup go2.launch.py
```

Now you should be able to connect to the Go2's network on your PC, open a terminal and run `ros2 topic list` and see the topics from this package, e.g. `/cmd_vel`.

If all of the above instructions were followed, and the below section on using `systemctl` has been completed, the robot can be used without needing to SSH into it. Simply power up the robot and leave it for a minute or so. Then connect to the WiFi hotspot and start the container. You should see all of the topics from this SDK listed and can begin working with the Go2 over WiFi with ROS2.

## Setting up the `systemctl` service

We can use systemctl to start this package automatically.

Open the file `auxiliary/go2-sdk.service` and ensure that the username is correct, and the path to the package is correct. The `ExecStart` field is the command that is executed. Ensure that all paths and the `WorkingDirectory` are correct. The `~/.ros2_env` file is used to source the ROS2 installation and other necessary packages like `cyclonedds` automatically. The required variables can be found in your `~/.bashrc` file.

Once the file has been formatted correctly, run the following commands to automatically start this at boot. 

```bash
sudo ln -s /home/unitree/ros2_ws/src/go2_robot/auxiliary/go2-sdk.service /etc/systemd/system/go2-sdk.service # Make a link to the file in the systemd folder
sudo systemctl daemon-reload # Makes systemd refresh the list of services
sudo systemctl enable go2-sdk.service # Start the service on boot
sudo systemctl start go2-sdk.service # Start the service now
```

The output can be checked by running

```bash
sudo systemctl status ros2_package.service
```

## Running with Docker

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

If the robot is not already running the `go2_bringup` package, it can be run from the container and the robot can be used.

```bash
ros2 launch go2_bringup go2.launch.py
```

If using the Docker container, it is easy to change the selected ROS2 distribution. When building the container, simply include the name of the distribution after the interface name, e.g. `./build.sh eth0 humble`.

## Running without Docker

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

_This document was written by Toby Godfrey (t.godfrey \~at\~ soton.ac.uk, @tgodfrey0 on GitHub)_
