# Slamcore ROS 2 Demos and Tutorials

<a href="https://docs.slamcore.com/nav2-integration.html">
<img alt="ros2-examples documentation" src="https://img.shields.io/badge/docs-docs.slamcore.com-orange"></a>
<a href="https://github.com/slamcore/slamcore-ros2-examples/blob/master/LICENSE.md" alt="LICENSE">
<img src="https://img.shields.io/github/license/slamcore/slamcore-ros2-examples.svg" /></a>
<a href="https://github.com/pre-commit/pre-commit">
<img src="https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit&logoColor=white" alt="pre-commit"></a>
<a href="https://github.com/psf/black">
<img alt="Code style: black" src="https://img.shields.io/badge/code%20style-black-000000.svg"></a>

This repository contains packages to go alongside our dedicated demonstration
tutorials which will allow you to get Slamcore up and running with ROS 2 Foxy or
Galactic in no time. The current examples demonstrate how to integrate
[Slamcore](https://www.slamcore.com/) with [Nav2](https://navigation.ros.org/),
the ROS 2 Navigation Stack, using a [RealSense D435i](https://www.intelrealsense.com/depth-camera-d435i/)
depth camera and one of the following robots:

- [Kobuki](https://robots.ros.org/kobuki/)
- [Create 3](https://edu.irobot.com/what-we-offer/create3)
- [TurtleBot 4 Standard or Lite](https://clearpathrobotics.com/turtlebot-4/)

> The TurtleBot 4 uses a Create 3 as the base. We have launch files to use our
software on the standalone Create 3 base, as well as on the TurtleBot 4 robots,
to take advantage of the additional features provided by the TurtleBot 4 software
packages.

We have tested these examples on the following hardware platforms:

- [Jetson Xavier NX](https://www.nvidia.com/en-gb/autonomous-machines/embedded-systems/jetson-xavier-nx/)
- [Raspberry Pi 4B 4GB](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/)

Nevertheless, by following our [Nav2 Integration
Tutorial](https://docs.slamcore.com/nav2-integration.html) you will be able to
modify the launch and config files in this repo to suit your setup.

## Supported Versions

- Kobuki
  - ROS 2 Foxy
  - Slamcore C++ API v23.01.48
  - Slamcore ROS 2 Wrapper v23.01.13
- Create 3
  - ROS 2 Galactic
  - Slamcore C++ API v23.01.48
  - Slamcore ROS 2 Wrapper v23.01.13
  - Create 3 Firmware G.4.1+
- TurtleBot 4 Standard/Lite
  - ROS 2 Galactic
  - Slamcore C++ API v23.01.48
  - Slamcore ROS 2 Wrapper v23.01.13
  - Create 3 Firmware G.4.1+

## Getting started

You have two options to set up your system for this demo:

1. Install the required Debian packages and setup your ROS workspace manually on
   an Ubuntu 20.04 system. See the `dependencies-<robot>.txt` for reference.
2. Run the demo inside a docker container. You can create it using the
   [Dockerfile](Dockerfile) and [Taskfile](https://taskfile.dev/) provided. This
   is suggested especially for devices running Ubuntu 18.04.

For detailed steps on how to integrate Slamcore's ROS 2 wrapper with Nav2 head
over to our [Nav2 Integration Tutorial](https://docs.slamcore.com/nav2-integration.html).
The tutorial will take you through all the steps from cloning this repository
and setting up dependencies (or Docker), to creating a map, navigating and
sending goals from a remote machine.

## About Slamcore

[Slamcore](https://www.slamcore.com/) offers commercial-grade visual-inertial
simultaneous localisation and mapping (SLAM) software for real-time autonomous
navigation on robots and drones.
