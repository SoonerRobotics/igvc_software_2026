#!/bin/bash

# General
sudo apt update

# Dependencies
sudo apt install git unzip curl cmake build-essential can-utils -y

# Audible Feedback
sudo apt install -y ffmpeg

# OpenCV
# On Jetson, use the version provided by NVIDIA (JetPack) or build with CUDA support.
# sudo apt install -y libopencv-dev

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

# TODO: Automatically add systemd services
# TODO: Automatically add udev rules

