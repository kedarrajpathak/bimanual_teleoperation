ARG ROS_DISTRO=humble
ARG RMW_IMPLEMENTATION=rmw_fastrtps_cpp
FROM osrf/ros:${ROS_DISTRO}-desktop
ENV ROS_DISTRO=${ROS_DISTRO}
ARG DEBIAN_FRONTEND=noninteractive

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# Install dependencies with apt
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y \
    ament-cmake \
    python3-pip \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-flake8 \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    wget &&\
    apt-get update --fix-missing && \
    apt-get dist-upgrade -y && \
    rosdep update -y

COPY ../colcon_ws/src/ /colcon_ws/src/
WORKDIR /colcon_ws/
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -r -y
RUN source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --parallel-workers 2
RUN source /colcon_ws/install/setup.bash

RUN pip install mediapipe opencv-contrib-python transforms3d quaternion flask websockets

RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y \
    ros-$ROS_DISTRO-topic-based-ros2-control \
    ros-$ROS_DISTRO-foxglove-bridge \
    ros-$ROS_DISTRO-tf-transformations \
    ffmpeg 

COPY ../overlay_ws/src/ /overlay_ws/src/
WORKDIR /overlay_ws/
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -r -y
RUN source /colcon_ws/install/setup.bash && colcon build
RUN source /overlay_ws/install/setup.bash

RUN echo "source /overlay_ws/install/setup.bash" >> ~/.bashrc