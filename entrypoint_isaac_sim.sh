#!/bin/bash
export DISPLAY=:1
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
# export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/isaac-sim/exts/omni.isaac.ros2_bridge/humble/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib

# ./python.sh /repos/kinova-humble/omniverse/isaac_moveit.py
bash /isaac-sim/isaac-sim.sh --allow-root 1 