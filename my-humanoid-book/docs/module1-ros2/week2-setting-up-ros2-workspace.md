---
id: week2-setting-up-ros2-workspace
title: 'Week 2: Setting up a ROS2 Workspace'
---

This week, we will learn how to set up a ROS2 workspace. A workspace is a directory containing ROS2 packages.

## Creating a Workspace

A ROS2 workspace is a directory with a specific structure. Inside the workspace, there is a `src` directory where the source code of the packages is located.

To create a workspace, you can use the following commands:

```bash
mkdir -p ros2_ws/src
cd ros2_ws
```

## Building a Workspace

Once you have a workspace with some packages, you can build it using `colcon`:

```bash
colcon build
```

`colcon` is a command line tool to build, test and use multiple software packages. It will build the packages in the `src` directory and install the executables and other artifacts in the `install` directory.

## Sourcing the Workspace

Before you can use the packages in your workspace, you need to source the `setup.bash` file in the `install` directory:

```bash
source install/setup.bash
```

This will add the packages in your workspace to your environment, so that you can use them.
