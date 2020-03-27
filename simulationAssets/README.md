# Simulation Assets

This folder contains:

1. launchFiles - launch files called by the startupScript  ```px4_launch_gazebo.sh```

2. models - pre-prepared iris models that include 8-Segment LiDARs and cameras
3. worlds - pre-prepared gazebo worlds with power line models
4. startupScripts - handy scripts to start

   1. PX4-SITL with ROS and Gazebo

   2. MAVROS for SITL

## Installation

From the terminal, run the command

```bash
./installAssets.sh /path/to/Firmware/Folder
```

This will install all the launch scripts, model files and world assets to the correct folders as shown below

- Launch Files -> /Firmware/launch

- Model Files -> /Firmware/Tools/sitl_gazebo/models
- World Files -> /Firmware/Tools/sitl_gazebo/worlds
