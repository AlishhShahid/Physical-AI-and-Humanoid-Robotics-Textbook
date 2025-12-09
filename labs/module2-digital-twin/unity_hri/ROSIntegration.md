# ROS-Unity Integration for Humanoid Robot Digital Twin

## Overview
This document describes how to connect the Unity visualization to the Gazebo simulation via ROS2 communication.

## Architecture
```
Gazebo Simulation ←→ ROS2 Bridge ←→ Unity TCP Connector ←→ Unity Visualization
```

## Implementation Approach

### 1. ROS-TCP-Connector Setup
The Unity project uses the ROS-TCP-Connector package to communicate with ROS2:

- Unity acts as a TCP client
- ROS2 bridge acts as a TCP server
- Messages are serialized/deserialized between ROS and Unity

### 2. Message Types Used
- **sensor_msgs/JointState**: For robot joint positions
- **sensor_msgs/Imu**: For IMU sensor data
- **geometry_msgs/TransformStamped**: For robot pose
- **std_msgs/String**: For command and status messages

### 3. Unity Scripts

#### RobotController.cs
```csharp
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;

public class RobotController : MonoBehaviour
{
    // ROS connection
    ROSConnection ros;

    // Robot joint transforms
    Dictionary<string, Transform> jointMap = new Dictionary<string, Transform>();

    // ROS topic
    string jointStateTopic = "/humanoid_robot/joint_states";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<JointStateMsg>(jointStateTopic, UpdateRobotPose);

        // Map joint names to transforms
        MapJoints();
    }

    void UpdateRobotPose(JointStateMsg jointState)
    {
        for (int i = 0; i < jointState.name.Array.Length; i++)
        {
            string jointName = jointState.name.Array[i];
            float jointPosition = (float)jointState.position[i];

            if (jointMap.ContainsKey(jointName))
            {
                // Apply joint position to Unity transform
                ApplyJointPosition(jointMap[jointName], jointPosition);
            }
        }
    }

    void MapJoints()
    {
        // Map each joint name to its corresponding transform in the Unity model
        jointMap["left_hip_joint"] = transform.Find("LeftHip");
        jointMap["left_knee_joint"] = transform.Find("LeftKnee");
        // ... continue for all joints
    }

    void ApplyJointPosition(Transform jointTransform, float position)
    {
        // Apply rotation based on joint position
        jointTransform.localRotation = Quaternion.Euler(0, 0, position * Mathf.Rad2Deg);
    }
}
```

### 4. Bridge Configuration
A ROS2 node would act as a bridge between Gazebo and Unity:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import String
import socket
import json

class UnityBridge(Node):
    def __init__(self):
        super().__init__('unity_bridge')

        # Subscribe to Gazebo joint states
        self.joint_sub = self.create_subscription(
            JointState,
            '/humanoid_robot/joint_states',
            self.joint_callback,
            10
        )

        # Subscribe to IMU data
        self.imu_sub = self.create_subscription(
            Imu,
            '/humanoid_robot/imu/data',
            self.imu_callback,
            10
        )

        # TCP server for Unity
        self.tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_server.bind(('localhost', 10000))
        self.tcp_server.listen(1)

        self.get_logger().info('Unity bridge initialized')

    def joint_callback(self, msg):
        # Forward joint state to Unity
        data = {
            'type': 'joint_state',
            'names': list(msg.name),
            'positions': list(msg.position),
            'velocities': list(msg.velocity),
            'efforts': list(msg.effort)
        }
        self.send_to_unity(data)

    def imu_callback(self, msg):
        # Forward IMU data to Unity
        data = {
            'type': 'imu',
            'orientation': [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w],
            'angular_velocity': [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            'linear_acceleration': [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        }
        self.send_to_unity(data)

    def send_to_unity(self, data):
        try:
            # Send JSON data to Unity
            conn, addr = self.tcp_server.accept()
            conn.send(json.dumps(data).encode())
            conn.close()
        except Exception as e:
            self.get_logger().error(f'Error sending to Unity: {e}')
```

### 5. Synchronization
- Unity visualization updates at 30-60 FPS
- ROS2 messages published at 50-100 Hz
- Interpolation used for smooth visualization between updates

### 6. Human-Robot Interaction
Unity can send commands back to the robot:
- Joint position commands
- Navigation goals
- High-level behaviors
- Emergency stops

## Testing
The integration can be tested by:
1. Running Gazebo simulation with the humanoid robot
2. Starting the ROS-Unity bridge
3. Launching the Unity visualization
4. Verifying that robot movements in Gazebo are reflected in Unity
5. Testing bidirectional communication

This architecture provides real-time visualization of the Gazebo simulation in Unity, enabling enhanced human-robot interaction capabilities.