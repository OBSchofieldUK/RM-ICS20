# RM-ICS20 Repository

Git Repository for use with Intelligent Collaborative System Design Elective at SDU (spring semester 2020)

Maintainer: Oscar Bowen Schofield (obs@mmmi.sdu.dk)

> Each Folder has its own respective README.md, be sure to check them out before asking questions.

## Dependencies and Recommended Software

Ensure the following are installed:

- Ubuntu 16.04/ 18.04 *(18.04 recommended)*
- [ROS](www.ros.org) Melodic/Kinetic *(Melodic recommended)*
- [PX4 Firmware](https://github.com/px4/firmware) *(V1.8.2 recommended)*
- [MAVROS](http://wiki.ros.org/mavros)
- [python-utm](http://pypi.python.org/pypi/utm)
- [QGroundControl](http://qgroundcontrol.com/) *(for debugging)*
- [terminator](https://gnometerminator.blogspot.com/p/introduction.html) *(recommended terminal package, **not explicitly required**)*

## Getting started

### Prerequisite

> Make sure you have followed the [PX4 toolchain installation](https://dev.px4.io/v1.9.0/en/setup/dev_env_linux.html#jmavsimgazebo-simulation) before continuing.

Clone the PX4 Firmware and build the [Gazebo-SITL configuration](https://dev.px4.io/v1.8.2/en/simulation/gazebo.html)

If the firmware builds and runs correctly, you should be able to run ```commander takeoff``` within the psh shell, causing the UAV to take off, then land.

### Recommended Folder Structure

``` tree
.
+-- PX4 Firmware
+-- catkin_workspace
|   +-- startupScripts
|   +-- src
|   |   +-- RM-ICS20 Repository
|   |   +-- **Additional nodes**
```

[Simulation Asset installation](https://github.com/OBSchofieldUK/RM-ICS20/tree/master/simulationAssets)

### Running Autonomy framework system

You will need four terminals to operate the system (may need more for development)

1. **PX4/Simulation window**
  
```bash
   cd catkin_workspace/startupScripts
   ./px4_launch_gazebo.sh
```

2. **MAVROS Window**

```bash
    cd catkin_workspace/startupScripts
    ./launch_mavROS.sh
```

If successful, the below text should be visible in the terminal

![Image](https://github.com/obschofielduk/RM-ICS20/blob/master/documentation/MAVROS_connected.png)

3. **droneFramework Window**

```bash
    cd catkin_workspace
    catkin build
    source devel/setup.bash
    roslaunch root_framework D4E_auto.launch

```

4. **User Input Window**

> This Assumes catkin_workspace has been previously built

```bash
    cd catkin_workspace
    source devel/setup.bash
    rosrun root_framework keypressNode.py
```
