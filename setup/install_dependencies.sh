#!/bin/bash

# General
sudo apt update

# Dependencies
sudo apt install git unzip curl cmake build-essential -y

# OpenCV
sudo apt install -y libopencv-dev

# Bluetooth
sudo apt install --reinstall \
  bluez \
  bluez-tools \
  linux-firmware \
  xboxdrv \
  steam-devices
