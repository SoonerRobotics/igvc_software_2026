#!/bin/bash

# General
sudo apt update

# Dependencies
sudo apt install git unzip curl cmake build-essential -y

# Audible Feedback
sudo apt install -y ffmpeg

# CAN Utils
sudo apt install -y can-utils

# OpenCV
sudo apt install -y libopencv-dev

# Camera stuff
sudo apt install -y v4l-utils

# Bluetooth
sudo apt install --reinstall \
  bluez \
  bluez-tools \
  linux-firmware \
  xboxdrv \
  steam-devices \
  pulseaudio \
  pulseaudio-module-bluetooth

# SDL
sudo apt install -y libsdl2-dev

# add user to video group
sudo adduser $USER_TO_ADD video
