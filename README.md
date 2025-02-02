# 2024 SEA:ME HACKATHON_autonomous-driving with jetracer


## setting


### Requirements

Jetracer

Jetson Nano

usb-cam x 2

Ultrasonic sensor - US015  X 3

2D Lidar sensor

Arduino uno


### Tools

Ubuntu 20.04

ROS noetic



## Overview

This project contains two main modules designed for autonomous driving on tracks and handling various driving missions.

# hough_drive

This module ensures fast track driving without lane departure using Hough Transform for lane detection.

# mission_drive

This module handles a sequence of missions including:

1. Lane keeping

2. Stopping at traffic lights

3. Navigating dynamic obstacles

4. Stopping at the stop line
