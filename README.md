# Drones4Energy - Powerline Landing
Maintainer: Oscar Bowen Schofield (obs@mmmi.sdu.dk)


This repo is for the ongoing development of Powerline guidance and Landing under the *Drones4Energy* Project at the University of Southern Denmark, Odense. 

The structure of this repo is as follows: 
- visionEstimation 
  - The processing of images to detect powerlines. 
  - Sensor fusion between image data, IMU and LiDAR data 
  - Estimating the 3D position of the powerline w.r.t the UAV. 
- droneFramework 
  - Backbone of the autonomous system to locate a power pylon within the local vicinity 
  - Provide basic navigation to a desired waypoints    
- simulationAssets
  - Drone and world models needed to run GazeboSim 
  - Pre-made bash scripts to launch the simulations properly
- systemDependencies
  - Packages and files used by both visionEstimation and droneFramework

# Dependencies
Each folder has its own README which need to be followed for the whole system to work, however the main dependencies are:

1. ROS      (Melodic/Kinetic)
2. OpenCV   (Tested with V3.3.1)
3. PX4      (Tested with V1.8.2) 
4. [eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page) Library 
   1. (Ubuntu)-> ```sudo apt-get install libeigen3-dev```
5. [RapidJSON](https://github.com/Tencent/rapidjson/) library 
   
# Getting Started

#TODO: *update in the near future*
