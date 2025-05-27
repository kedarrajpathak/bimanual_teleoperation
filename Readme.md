# Bimanual Teleoperation
## About
This repo provides two packages to facilitate teleoperation of dual arm setup using moveit2.

- The first package **`dual_arm_kinova_moveit_config`** is based on the following two moveit2 packages:

  [dual_arm_panda_moveit_config](https://github.com/moveit/moveit_resources/tree/ros2/dual_arm_panda_moveit_config)

  [moveit_servo](https://github.com/moveit/moveit2/tree/main/moveit_ros/moveit_servo)

- The second package **`aruco_pose_estimation`** is based on the official tutorial of OpenCV

  [Detection of ArUco Markers](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)

## Installation

MoveIt2 is built from source from the `main` branch. The relevant repositories are added as submodules in this git repository. Execute the following commands to clone all the submodules and then build the docker image.

- `git submodule update --init --recursive`

- `bash docker_build.sh`

Once the image is built open the folder in devcontainer from vscode.


## Usage

- run `xhost +` before any command and execute following commands each in a new termianl

- ```sh
  ros2 launch dual_arm_kinova_moveit_config move_group_kinova.launch.py robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true

  ros2 launch aruco_pose_estimation aruco_pose_estimation.launch.py
  ```