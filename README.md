# Robot Programming course project: ICP Localization
This repository contains the project for the **Robot Programming exam**. It implements an **ICP-based localization** system using laser scan data to estimate the robot's pose within a known map in **ROS**.

## Requirements

Ensure that **ROS Noetic** is installed before proceeding.

### Map server

Install the ros-noetic-map-server package:

```sh
sudo apt install ros-noetic-map-server
```

### Stage-ROS

Install the ros-noetic-stage-ros and ros-noetic-teleop-twist-keyboard packages:

```sh
sudo apt install ros-noetic-stage-ros ros-noetic-teleop-twist-keyboard
```

## Setting up the workspace

Create a **ROS workspace** and clone the repository:

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

## Running the project

**Before launching any node**, open a terminal, navigate to the workspace, and **source the environment**:

```sh
cd ~/ROS_workspaces/icp_ws/
source devel/setup.bash
```

**Terminal 1.**   Start the ROS master

  ```sh
  roscore
  ```

**Terminal 2.**  Run the map server

  ```sh
  cd src/02_icp_localization
  rosrun map_server map_server test_data/cappero_map.yaml
  ```

**Terminal 3.**  Launch RViz with the predefined configuration

  ```sh
  cd src/02_icp_localization
  rviz -d test_data/rviz.rviz
  ```

**Terminal 4.**   Start the simulator

  ```sh
  rosrun stage_ros stageros test_data/cappero.world
  ```

**Terminal 5.**   Run the localization node

  ```sh
  rosrun icp_localization localizer_node
  ```

### Testing

In **RViz**, set the **initial pose** of the robot close to its actual position.
Use the simulator to move the robot and observe how the localization updates in real time.

