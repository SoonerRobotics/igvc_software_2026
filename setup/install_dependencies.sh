#!/bin/bash

# General
sudo apt update

# Dependencies
sudo apt install git unzip curl cmake build-essential -y

# Audible Feedback
sudo apt install -y ffmpeg

# OpenCV
sudo apt install -y libopencv-dev

# SDL
sudo apt install -y libsdl2-dev

# TODO: Automatically add systemd services
# TODO: Automatically add udev rules