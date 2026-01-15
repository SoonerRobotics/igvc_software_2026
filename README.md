# IGVC Software 2026

This repository contains the full software stack for our IGVC 2026 entry, Suspended Disbelief. This includes the robot control software, communication schemas, and firmware.

## Overview

```bash
├── firmware # Embedded firmware for microcontrollers and similar devices
├── igvc_csharp # Core code that controls the robot
├── igvc_flatbuffers # Shared communication schemas between the frontend, the core, and the simulator
├── igvc_gui # The user interface for the robot
└── setup # Helper scripts for setting up a machine to run the robot
```

## Getting Started

### Cloning

```bash
git clone https://github.com/SoonerRobotics/igvc_software_2026.git
cd igvc_software_2026
```

### Setup

To setup and install any dependencies, only one command is needed. This will install all required software, including building OpenCv from scratch. This may take a little while to run/build depending on your machines specs.
```bash
./setup/install.sh
```

## Project Breakdown

### `firmware/` - Firmware

The [firmware/](firmware/) folder contains firmware for all of our microcontrollers and devices. The following tree gives a short description of each. Each individual folder, e.g. [firmware/can_controller](firmware/can_controller), contains their own README file which describes the firmware and any special tools needed to build/deploy it.

```bash
firmware/
├── can_converter # Converts from the REV canspec to our canspec and vice versa
├── estop_receiver # Listens for EStop signals and motor instructions sent by the EStop Remote
├── estop_remote # Sends EStop signals and motor instructions to the EStop Receiver
├── fan_controller # Controls fans in the robot based on temperature curves and emits telemetry
├── hub # Routes power, ground, and other signals around the robot and emits per device voltage/current telemtry
└── safety_lights # Controls the safety lights on the robot
```

### `igvc_csharp/` - Core Robot Code

The [igvc_sharp/](igvc_csharp/) folder contains all of the core robot code, including: vision, hardware communication, simulator support, etc. In the past this was written in a mix of Python and C++ using ROS2. This year, we have transitioned to using C# as a mega prgoram that multithreads everything. This should make onboarding new members easier due to both the similarity to Java and lack of needing to learn how ROS works. A basic file structure is shown below, but a more indepth [README](igvc_csharp/README.md) file exists in the folder.

### `igvc_flatbuffers` - Shared Communication Schema

### `igvc_gui/` - Robot User Interface

### `setup/` - Setup and Install Scripts