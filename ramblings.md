# Zemlin Ramblings

This document technically contains the full working theory of how the IGVC 2026 robot is going to function for both AutoNav and Self Drive, althouh it should all be taken with fairly sizable grains of salt

## Architecture

The robot will be created using ROS2 Jazzy on Ubuntu 24.04. The majority of the code will be in C++ for both performance reasons and to keep everything similarly structured throughout the entire codebase. In the event that something is overly complicated in C++ or will require more effort than its worth to port into C++, we will use Python.

## Sensors

### Cameras

The robot will be using two SVPRO Usb Cameras, the exact same model used on Twistopher.  These work well, have a decent field of view, support the resolution we need (720p), and are global shutter so we don't get any weird blurring as the robot moves.

### GPS

The robot will be using a VectorNav GPS (The VN200 if I recall correctly). The primary data we will be using from this is the latitude and longitude, but it also provides some other information such as heading which we might use.

## Systems

This section will cover all of the "systems" that will be running within the ROS stack, such as our particle filter, vision tranfsormation nodes (e.g. hsv thresholding), etc.

## Motor Control
