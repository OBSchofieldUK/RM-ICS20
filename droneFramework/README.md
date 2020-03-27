# Drone Framework 

The drone framework is the backbone of the autonomous operations on the UAV.
This folder is split up into three parts:

- root_framework, the core framework, including droneCore, messageHandler, and userInput
- secondary pilots, which are responsible for an individual operation (loiter, examplePilot)
- peripheralControl - this includes the conversion script to format the gazeboSim 360-degree rangefinder into a LeddarTech VU8 8-segment LiDAR.

## How to use the framework

>Make sure to add the repo to your catkin workspace, catkin build, source devel/setup.bash first!

### Entire Framework

To launch the entire "root framework", use the command:

`roslaunch root_framework D4E_auto.launch`

Operating the system requires a separate terminal, running the `keypressNode` python script

`rosrun root_framework keypressNode.py`

To control, click into the keypressNode terminal and input the keys shown below

| Keypress | Action |
|:-----:|:--------|
|t | takeoff + offboard + loiter mode |
|o | offboard + loiter (useful if something stops during operation) |
|l | enable loiter |
|wasd | Loiterpilot: forwards, back, left, right |
|qe | Loiterpilot: yaw ccw/cw (respectively) |
|zx | decrease/increase altitude |
|r | move drone to powerline set (fancy model) |
|f | move drone to single powerline (accurate line thickness) |
|h | move the drone to hover above "home" at 7.5m altitude |

<!--### LoiterMode

Whilst in loiter mode, you can use the keyboard controls to move the the drone 
`wasd`  for XY moves
`zx`    for up/down
`qe`    for rotation
-->

### messageControl

Messages handler for all pilots, converting and sending a single pilot output to MAVROS GPS/Attitude setpoints

## Adding a new pilot

To add a new pilot, you will need to:

1. droneCore = add a new state under _onKeyboardPress

2. messageControl = add a new subscriber with topic `/onboard/setpoint/{YourPilotHere}`

3. Publishers and subscribers
   1. Publish calculated setpoints to above `/onboard/setpoint/{YourPilotHere}`.

   2. Subscribe to `'/onboard/state'` to enable the pilot.
