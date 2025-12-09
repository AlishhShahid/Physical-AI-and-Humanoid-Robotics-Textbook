#!/bin/bash

# This script builds the ROS2 workspace for the humanoid control lab on a Jetson device.

echo "Building ROS2 workspace..."
cd ../ros2_ws
colcon build
echo "Build complete."
