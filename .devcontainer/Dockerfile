ARG ROS_DISTRO=humble
ARG RMW_IMPLEMENTATION=rmw_fastrtps_cpp
FROM osrf/ros:${ROS_DISTRO}-desktop
ENV ROS_DISTRO=${ROS_DISTRO}
ARG DEBIAN_FRONTEND=noninteractive

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

#### GPG key issue fix. Temporary, to be removed when the issue is resolved.
RUN rm /etc/apt/sources.list.d/ros2-latest.list \
    && rm /usr/share/keyrings/ros2-latest-archive-keyring.gpg

RUN apt-get update \
    && apt-get install -y ca-certificates curl

RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') ;\
    curl -L -s -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
    && apt-get update \
    && apt-get install /tmp/ros2-apt-source.deb \
    && rm -f /tmp/ros2-apt-source.deb
####

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