# Setting Up Your ROS 2 Workspace

## Prerequisites

Before we begin building our humanoid robot's nervous system, you'll need to set up your development environment. This chapter will guide you through the process of installing ROS 2 and creating your workspace.

## Installing ROS 2

For this textbook, we recommend using ROS 2 Humble Hawksbill (LTS version) which provides long-term support and stability. Follow these steps based on your operating system:

### Ubuntu (Recommended)
```bash
# Add ROS 2 repository
sudo apt update && sudo apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS 2 Humble
sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y python3-rosdep2
sudo apt install -y python3-colcon-common-extensions

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS 2 environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Windows (Alternative)
Use WSL2 (Windows Subsystem for Linux) with Ubuntu for the best ROS 2 experience on Windows.

## Creating Your Workspace

Now that ROS 2 is installed, let's create a workspace for our humanoid robot project:

```bash
# Create the workspace directory
mkdir -p ~/humanoid_ws/src
cd ~/humanoid_ws

# Create the source directory if it doesn't exist
mkdir -p src
```

## Building Your First Package

In the previous chapter, we outlined the components of our humanoid robot's nervous system. Now we'll create the main package:

```bash
# Navigate to the source directory
cd ~/humanoid_ws/src

# Create the humanoid control package using ros2 pkg
ros2 pkg create --build-type ament_python humanoid_control_pkg --dependencies rclpy std_msgs sensor_msgs geometry_msgs
```

This creates the basic structure for our package. However, in our project, we've already created the `humanoid_control_pkg` with more specific functionality. Let's look at what we've built so far:

## Project Structure

Our humanoid robot workspace follows this structure:

```
labs/
├── module1-ros2/
│   └── ros2_ws/
│       └── src/
│           ├── humanoid_control_pkg/      # Main control package
│           │   ├── humanoid_control_pkg/
│           │   │   ├── __init__.py
│           │   │   ├── humanoid_controller.py
│           │   │   ├── basic_communication.py
│           │   │   ├── rclpy_examples.py
│           │   │   └── motor_driver.py
│           │   ├── package.xml
│           │   ├── setup.py
│           │   └── test/
│           │       ├── test_humanoid_controller.py
│           │       ├── test_pytest.py
│           │       └── conftest.py
│           └── humanoid_description/      # Robot description package
│               ├── humanoid.urdf
│               ├── package.xml
│               ├── launch/
│               │   └── display_robot.launch.py
│               └── config/
│                   └── rviz_config.rviz
```

## Building the Workspace

To build your workspace:

```bash
cd ~/humanoid_ws
colcon build --packages-select humanoid_control_pkg humanoid_description
source install/setup.bash
```

## Running Your First Nodes

Once built, you can run any of the nodes we've created:

```bash
# Source the workspace
source ~/humanoid_ws/install/setup.bash

# Run the humanoid controller
ros2 run humanoid_control_pkg humanoid_controller

# In another terminal, run the joint state publisher
ros2 run humanoid_control_pkg joint_state_publisher

# Or run the motor driver
ros2 run humanoid_control_pkg motor_driver
```

## Launch Files

ROS 2 provides launch files to start multiple nodes at once. For our robot, we can create a launch file to start all essential nodes:

```bash
# Launch the complete robot system
ros2 launch humanoid_description display_robot.launch.py
```

## Testing Your Setup

To verify everything is working correctly:

1. Make sure your workspace is sourced: `source ~/humanoid_ws/install/setup.bash`
2. Check available nodes: `ros2 node list`
3. Check available topics: `ros2 topic list`
4. Echo a topic to see data: `ros2 topic echo /joint_states sensor_msgs/msg/JointState`

In the next chapter, we'll dive deeper into creating custom ROS 2 nodes for our humanoid robot's specific needs.