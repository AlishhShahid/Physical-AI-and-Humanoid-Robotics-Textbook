# Deploying Module 1 on NVIDIA Jetson

This guide provides instructions for deploying and running the ROS2 humanoid control lab on an NVIDIA Jetson device.

## Prerequisites

1.  An NVIDIA Jetson device (e.g., Jetson Nano, Jetson Xavier) flashed with the latest JetPack SDK.
2.  ROS2 installed on the Jetson.
3.  This repository cloned onto the Jetson.

## Build Instructions

1.  **Navigate to the ROS2 workspace:**
    ```bash
    cd labs/module1-ros2/ros2_ws
    ```

2.  **Build the workspace:**
    ```bash
    colcon build
    ```

## Run Instructions

1.  **Source the workspace:**
    ```bash
    source install/setup.bash
    ```

2.  **Launch the humanoid control package:**
    ```bash
    ros2 launch humanoid_control_pkg humanoid_control.launch.py
    ```
