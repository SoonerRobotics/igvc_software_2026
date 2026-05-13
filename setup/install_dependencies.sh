#!/bin/bash

# General
sudo apt update

# Dependencies
sudo apt install git unzip curl cmake build-essential -y

# Audible Feedback
sudo apt install -y ffmpeg

# OpenCV
sudo apt install -y libopencv-dev

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

