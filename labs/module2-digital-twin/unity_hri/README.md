# Unity Human-Robot Interaction Project

This directory contains the Unity project for human-robot interaction visualization of the humanoid robot digital twin.

## Project Structure
```
Assets/
├── Scenes/                 # Unity scenes
├── Scripts/                # C# scripts for robot control
├── Models/                 # 3D models for the robot
├── Materials/              # Material definitions
├── Prefabs/                # Robot and environment prefabs
└── Plugins/                # ROS integration plugins
```

## Setup Instructions

1. Install Unity Hub and Unity 2022.3 LTS or later
2. Install the ROS-TCP-Connector package for Unity
3. Import the humanoid robot model
4. Configure the TCP connection to communicate with ROS2

## ROS Integration

The Unity project communicates with ROS2 via TCP/IP using the ROS-TCP-Connector package. This allows:

- Real-time visualization of robot joint states
- Human-robot interaction through Unity UI
- Teleoperation capabilities
- Simulation data exchange

## Key Components

- **RobotController.cs**: Handles ROS message subscription and robot visualization
- **JointStateSubscriber.cs**: Processes joint state messages from ROS
- **HumanInteraction.cs**: Manages user input for robot control
- **SimulationBridge.cs**: Coordinates between Unity physics and ROS simulation

## Implementation Notes

This project would require:
- Unity Robotics Package for ROS integration
- Proper URDF import using Unity's URDF Importer
- Custom scripts for real-time joint state visualization
- TCP/IP communication layer to connect with ROS2

## Building

1. Open the project in Unity
2. Build for your target platform (Windows, Linux, etc.)
3. Configure ROS2 bridge connection settings
4. Run alongside Gazebo simulation

For complete implementation, this would need to be developed in the Unity Editor with proper 3D models and physics integration.