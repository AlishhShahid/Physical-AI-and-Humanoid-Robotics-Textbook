#!/bin/bash

# This script runs the tests for the humanoid_control_pkg.

echo "Running tests for Module 1..."
cd ros2_ws
colcon test
