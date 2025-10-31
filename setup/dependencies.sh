#!/bin/bash
set -e
ROS_DISTRO=jazzy

# OpenCV
sudo apt install libopencv-dev python3-opencv

# OpenCV Bridge
sudo apt install ros-$ROS_DISTRO-cv-bridge

# SocketCAN
sudo apt install libsocketcan-dev