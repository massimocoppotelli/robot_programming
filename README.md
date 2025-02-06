# Robot Programming course project: ICP Localization
This repository contains the project for the Robot Programming exam. It implements an ICP-based localization system using laser scan data to estimate the robot's pose within a known map in ROS.

## Requirements

Ensure that ROS Noetic is installed before proceeding.

### 1. Map server

Install the ros-noetic-map-server package:

```sh
sudo apt install ros-noetic-map-server
```

### 2. Stage-ROS

Install the ros-noetic-stage-ros and ros-noetic-teleop-twist-keyboard packages:

```sh
sudo apt install ros-noetic-stage-ros ros-noetic-teleop-twist-keyboard
```

### 3. Setting up the workspace

Create a ROS workspace and clone the repository:

```sh
cd ~
mkdir -p ROS_workspaces/icp_ws/src
cd ROS_workspaces/icp_ws/src
catkin_init_workspace
git clone https://github.com/massimocoppotelli/robot_programming.git
```

Build the workspace:

```sh
cd ~/icp_ws/
catkin_make
```

