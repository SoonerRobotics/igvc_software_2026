#!/bin/bash

# General
sudo apt update

# Dependencies
sudo apt install git unzip curl cmake build-essential -y

# OpenCV
sudo apt install -y libopencv-dev

# SDL
sudo apt install libsdl2-dev

# Bluetooth
#sudo apt install --reinstall \
#  bluez \
#  bluez-tools \
#  linux-firmware
#  xboxdrv \
#  steam-devices

# Xbox
#sudo apt install -y xboxdrv
