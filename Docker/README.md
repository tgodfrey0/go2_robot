# Running with Docker

To make it easier to get started with this SDK on a Go2 robot, a comprehensive Dockerile was created. The container supports window forwarding allowing for the use of GUI-based tools like `rviz2`.

To build the Docker container, run

```bash
./build.sh INTERFACE
```

where `INTERFACE` is the network interface which the robot is connected to. For example, if the Go2 was connected over ethernet and the interface was `eth0`, one would run `./build eth0`.

To the run the container, simply run

```bash
./run.sh
```

You will now be in the container and should be able to see the topics from the robot. This can be verified by running `ros2 topic list`.

The SDK can now be run and the robot can be used.

```bash
ros2 launch go2_bringup go2.launch.py
```
