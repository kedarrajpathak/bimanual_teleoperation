# Bimanual Teleoperation
## About
This repo provides two packages to facilitate teleoperation of dual arm setup using moveit2.

- The packages **`dual_arm_kinova_moveit_config`** and **`dual_arm_servo_moveit_config`** are based on the following moveit2 packages:

  [dual_arm_panda_moveit_config](https://github.com/moveit/moveit_resources/tree/ros2/dual_arm_panda_moveit_config)

  [moveit_servo](https://github.com/moveit/moveit2/tree/main/moveit_ros/moveit_servo)

  [ros2_kortex](https://github.com/Kinovarobotics/ros2_kortex.git)

- The packages **`aruco_pose_estimation`** and **`aruco_cube_tracking`** are based on the official tutorial of OpenCV

  [Detection of ArUco Markers](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)

## Installation

MoveIt2 is built from source from the `main` branch. The relevant repositories are added as submodules in this git repository. Execute the following commands to clone all the submodules and then build the docker image.

- `git submodule update --init --recursive`

- `bash docker_build.sh`

Once the image is built open the folder in devcontainer from vscode.

## Demo

### Dual Arm Teleoperation Demo

![Teleoperation using ArUco cubes](docs/aruco_cubes.gif)

### ArUco Pose Estimation Demo

![Teleoperation using ArUco markers](docs/aruco_markers.gif)

## Usage

- run `xhost +` before any command and execute following commands each in a new termianl

- ```sh
  ros2 launch dual_arm_kinova_moveit_config move_group_kinova.launch.py robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true

  # To start estimation and tracking of poses of individual markers
  ros2 launch aruco_pose_estimation aruco_pose_estimation.launch.py

  # To start estimation and tracking of poses of cubes with ArUco markers on its faces
  ros2 launch aruco_cube_estimation aruco_cube_estimation.launch.py
  ```