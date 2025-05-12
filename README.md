# mujoco_ros2_control with universal robot "ur5e and robotiq gripper

This repository provides a working simulation setup for a UR5e robot with a Robotiq 2F-85 gripper using `ros2_control` and MuJoCo. It is based on the [moveit/mujoco_ros2_control](https://github.com/moveit/mujoco_ros2_control) framework.

## Features

- UR5e + Robotiq 2F-85 model using XML and URDF/XACRO
- `mujoco_ros2_control` integration for controlling joints
- MuJoCo as the physics engine
- Launch file to run the full simulation and controllers

## Requirements

- ROS 2 (tested with Humble)
- MuJoCo 3.x
- `mujoco_ros2_control`


## Installation
### Clone the repository
```bash
mkdir -p ~/mujoco_ros2_ws/src/mujoco_ros2_ur
cd ~/mujoco_ros2_ws/src/mujoco_ros2_ur
git clone https://github.com/abuibaid/mujoco_ros2_control_ur_robotiq.git
cd ..
git clone https://github.com/moveit/mujoco_ros2_control/tree/main
```
###   Set MuJoCo environment
```bash
export MUJOCO_DIR=/path/to/mujoco-3.x.x
```
### Install dependencies, build the workspace and source it
```bash
cd ~/mujoco_ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build 
source install/setup.bash
```
## Launch Simulation
```bash
ros2 launch mujoco_ros2_ur ur5e.launch.py
```