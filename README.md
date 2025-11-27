# Custom Hardware Interface
Implemented a basic home position movement of a robot in RViz2 and MoveIt2 using custom hardware interface RobotCANHardware and a custom controller RobotController. 
This repository contains the launch file, the moveit package including the urdf

## Installation

### Clone the repository
Clone the repository
```
git clone https://github.com/Ath0601/CustomHardwareInterface.git
```
This will download the entire repository including the MoveIt package.

### Run the simulation
Launch the simulation using 
```
ros2 launch bringup robot_can.launch.py
```
Add the robot in the simulation using MotionPlanning tab in the Add section of RViz.
In the Plannning tab of the MotionPlanning, set the target goal to home position.
Click Plan & Execute
The robot will plan and execute the motion using the RobotCANHardware

### Check the hardware interface used
Check the hardware interface used
```
ros2 control list_hardware_components
```

## Video Demo
The video is attached below

https://github.com/user-attachments/assets/12cb32fd-b12e-432a-b0ae-1ac819522493

