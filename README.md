# SLAMcore ROS2 Demos and Tutorials

<a href="https://docs.slamcore.com/nav2-integration.html">
<img alt="ros2-examples documentation" src="https://img.shields.io/badge/docs-docs.slamcore.com-orange"></a>
<a href="https://github.com/bergercookie/taskwarrior-syncall/blob/master/LICENSE" alt="LICENSE">
<img src="https://img.shields.io/github/license/slamcore/slamcore-ros2-examples.svg" /></a>
<a href="https://github.com/pre-commit/pre-commit">
<img src="https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit&logoColor=white" alt="pre-commit"></a>
<a href="https://github.com/psf/black">
<img alt="Code style: black" src="https://img.shields.io/badge/code%20style-black-000000.svg"></a>

This repository contains packages to go alongside our dedicated demonstration
tutorials which will allow you to get SLAMcore up and running with ROS2 Foxy in
no time. The current example demonstrates how to integrate
[SLAMcore](https://www.slamcore.com/) with [Nav2](https://navigation.ros.org/),
the ROS2 Navigation Stack and was designed for a
[Kobuki](http://kobuki.yujinrobot.com/about2/) mobile base, with a [Jetson
Xavier
NX](https://www.nvidia.com/en-gb/autonomous-machines/embedded-systems/jetson-xavier-nx/)
and [RealSense D435i](https://www.intelrealsense.com/depth-camera-d435i/) depth
camera. Nevertheless, by following our [Navigation Stack Integration
Tutorial](https://docs.slamcore.com/nav2-integration.html) you will be able to
modify the launch and config files in this repo to suit your setup.

## About SLAMcore

[SLAMcore](https://www.slamcore.com/) offers commercial-grade visual-inertial
simultaneous localisation and mapping (SLAM) software for real-time autonomous
navigation on robots and drones. [Request
access](https://www.slamcore.com/sdk-access) today to get started.

## Getting started

You have two options to setup your system for these demo:

1. Run the demo inside a docker container. You can create it using the
   [Dockerfile](Dockerfile) and [Makefile](Makefile) provided. This is
   suggested especially if you are using NVIDIA's Xavier NX where Ubuntu 20.04
   is not supported out of the box.
2. Install the required Debian packages and setup your ROS workspace manually.
   See the [Dockerfile](Dockerfile) for reference.

Regardless of preference of setup, the next step is to learn how to integrate
SLAMcore's ROS2 wrapper with Nav2. To do this head over to the [Nav2 Integration
Tutorial](https://docs.slamcore.com/nav2-integration.html). The tutorial
will take you through all the steps from cloning this repository and setting up
dependencies to creating a map, navigating and sending goals from
a remote machine.
