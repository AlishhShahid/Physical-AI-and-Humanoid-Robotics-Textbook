---
sidebar_position: 1
---

# Week 6: Gazebo Simulation Fundamentals for Humanoid Robotics

## Learning Objectives

By the end of this week, you will be able to:
- Set up and configure Gazebo simulation environments for humanoid robots
- Create realistic physics models with proper collision and inertial properties
- Implement sensor simulation for IMU, cameras, and other humanoid sensors
- Configure robot controllers for simulation
- Debug and optimize simulation performance

## Introduction to Gazebo for Humanoid Robotics

Gazebo is a powerful physics-based simulation environment that provides realistic simulation capabilities for humanoid robots. It allows developers to test robot behaviors, algorithms, and control systems in a safe, repeatable environment before deploying to real hardware.

### Gazebo Architecture for Humanoid Robots

The Gazebo simulation environment consists of several key components when applied to humanoid robotics:

1. **Physics Engine**: Handles collision detection, contact forces, and rigid body dynamics
2. **Sensor Simulation**: Provides realistic sensor data including IMU, cameras, LIDAR, etc.
3. **Visualization**: 3D rendering of the robot and environment
4. **Plugin System**: Extensible architecture for custom robot controllers and sensors
5. **ROS2 Integration**: Seamless communication between simulation and ROS2 nodes

## Gazebo World Configuration

A well-configured world file is essential for humanoid robot simulation:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="humanoid_world">
    <!-- Include default environment -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Custom humanoid testing environment -->
    <model name="testing_platform">
      <pose>0 0 0 0 0 0</pose>
      <link name="platform_link">
        <collision name="platform_collision">
          <geometry>
            <box>
              <size>4.0 4.0 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="platform_visual">
          <geometry>
            <box>
              <size>4.0 4.0 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.6 0.6 0.6 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>100.0</mass>
          <inertia>
            <ixx>100.0</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>100.0</iyy>
            <iyz>0.0</iyz>
            <izz>100.0</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Add obstacles for navigation testing -->
    <model name="obstacle_1">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="obstacle_link">
        <collision name="obstacle_collision">
          <geometry>
            <box>
              <size>0.5 0.5 1.0</size>
            </box>
          </geometry>
        </collision>
        <visual name="obstacle_visual">
          <geometry>
            <box>
              <size>0.5 0.5 1.0</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
            <diffuse>0.8 0.2 0.2 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>5.0</mass>
          <inertia>
            <ixx>1.0</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>1.0</iyy>
            <iyz>0.0</iyz>
            <izz>1.0</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Physics configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>
  </world>
</sdf>
```

## URDF Enhancements for Gazebo Simulation

To make humanoid robots work properly in Gazebo, we need to enhance the URDF with Gazebo-specific elements:

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Include gazebo elements -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/humanoid_robot</robotNamespace>
      <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
    </plugin>
  </gazebo>

  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0.5"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.5"/>
      <geometry>
        <box size="0.3 0.3 1.0"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.5"/>
      <geometry>
        <box size="0.3 0.3 1.0"/>
      </geometry>
    </collision>
  </link>

  <!-- Left leg -->
  <joint name="left_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_hip_link"/>
    <origin xyz="0 0.1 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="3.0"/>
  </joint>

  <link name="left_hip_link">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 -0.25"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.25"/>
      <geometry>
        <cylinder radius="0.05" length="0.5"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25"/>
      <geometry>
        <cylinder radius="0.05" length="0.5"/>
      </geometry>
    </collision>
  </link>

  <!-- Gazebo material for left hip -->
  <gazebo reference="left_hip_link">
    <material>Gazebo/Blue</material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
    <kp>1000000.0</kp>
    <kd>1.0</kd>
  </gazebo>

  <!-- Right leg -->
  <joint name="right_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="right_hip_link"/>
    <origin xyz="0 -0.1 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="3.0"/>
  </joint>

  <link name="right_hip_link">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 -0.25"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.25"/>
      <geometry>
        <cylinder radius="0.05" length="0.5"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25"/>
      <geometry>
        <cylinder radius="0.05" length="0.5"/>
      </geometry>
    </collision>
  </link>

  <!-- Gazebo material for right hip -->
  <gazebo reference="right_hip_link">
    <material>Gazebo/Green</material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
    <kp>1000000.0</kp>
    <kd>1.0</kd>
  </gazebo>

  <!-- Left knee -->
  <joint name="left_knee_joint" type="revolute">
    <parent link="left_hip_link"/>
    <child link="left_knee_link"/>
    <origin xyz="0 0 -0.5"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="0.5" effort="100" velocity="3.0"/>
  </joint>

  <link name="left_knee_link">
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.2"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.05"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.2"/>
      <geometry>
        <cylinder radius="0.04" length="0.4"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2"/>
      <geometry>
        <cylinder radius="0.04" length="0.4"/>
      </geometry>
    </collision>
  </link>

  <!-- Gazebo material for left knee -->
  <gazebo reference="left_knee_link">
    <material>Gazebo/Blue</material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
    <kp>1000000.0</kp>
    <kd>1.0</kd>
  </gazebo>

  <!-- Right knee -->
  <joint name="right_knee_joint" type="revolute">
    <parent link="right_hip_link"/>
    <child link="right_knee_link"/>
    <origin xyz="0 0 -0.5"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="0.5" effort="100" velocity="3.0"/>
  </joint>

  <link name="right_knee_link">
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.2"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.05"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.2"/>
      <geometry>
        <cylinder radius="0.04" length="0.4"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2"/>
      <geometry>
        <cylinder radius="0.04" length="0.4"/>
      </geometry>
    </collision>
  </link>

  <!-- Gazebo material for right knee -->
  <gazebo reference="right_knee_link">
    <material>Gazebo/Green</material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
    <kp>1000000.0</kp>
    <kd>1.0</kd>
  </gazebo>

  <!-- Left ankle -->
  <joint name="left_ankle_joint" type="revolute">
    <parent link="left_knee_link"/>
    <child link="left_ankle_link"/>
    <origin xyz="0 0 -0.4"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.5" upper="0.5" effort="50" velocity="2.0"/>
  </joint>

  <link name="left_ankle_link">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.1"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.1"/>
      <geometry>
        <box size="0.1 0.08 0.2"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 -0.1"/>
      <geometry>
        <box size="0.1 0.08 0.2"/>
      </geometry>
    </collision>
  </link>

  <!-- Gazebo material for left ankle -->
  <gazebo reference="left_ankle_link">
    <material>Gazebo/Blue</material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
    <kp>1000000.0</kp>
    <kd>1.0</kd>
  </gazebo>

  <!-- Right ankle -->
  <joint name="right_ankle_joint" type="revolute">
    <parent link="right_knee_link"/>
    <child link="right_ankle_link"/>
    <origin xyz="0 0 -0.4"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.5" upper="0.5" effort="50" velocity="2.0"/>
  </joint>

  <link name="right_ankle_link">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.1"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.1"/>
      <geometry>
        <box size="0.1 0.08 0.2"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 -0.1"/>
      <geometry>
        <box size="0.1 0.08 0.2"/>
      </geometry>
    </collision>
  </link>

  <!-- Gazebo material for right ankle -->
  <gazebo reference="right_ankle_link">
    <material>Gazebo/Green</material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
    <kp>1000000.0</kp>
    <kd>1.0</kd>
  </gazebo>

  <!-- Left arm -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_shoulder_link"/>
    <origin xyz="0 0.15 0.8"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="3.0"/>
  </joint>

  <link name="left_shoulder_link">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder radius="0.03" length="0.3"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder radius="0.03" length="0.3"/>
      </geometry>
    </collision>
  </link>

  <!-- Gazebo material for left shoulder -->
  <gazebo reference="left_shoulder_link">
    <material>Gazebo/Orange</material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
    <kp>1000000.0</kp>
    <kd>1.0</kd>
  </gazebo>

  <!-- Left elbow -->
  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_shoulder_link"/>
    <child link="left_elbow_link"/>
    <origin xyz="0 0 -0.3"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="0.5" effort="30" velocity="3.0"/>
  </joint>

  <link name="left_elbow_link">
    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder radius="0.025" length="0.3"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder radius="0.025" length="0.3"/>
      </geometry>
    </collision>
  </link>

  <!-- Gazebo material for left elbow -->
  <gazebo reference="left_elbow_link">
    <material>Gazebo/Orange</material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
    <kp>1000000.0</kp>
    <kd>1.0</kd>
  </gazebo>

  <!-- Right arm -->
  <joint name="right_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="right_shoulder_link"/>
    <origin xyz="0 -0.15 0.8"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="3.0"/>
  </joint>

  <link name="right_shoulder_link">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder radius="0.03" length="0.3"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder radius="0.03" length="0.3"/>
      </geometry>
    </collision>
  </link>

  <!-- Gazebo material for right shoulder -->
  <gazebo reference="right_shoulder_link">
    <material>Gazebo/Orange</material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
    <kp>1000000.0</kp>
    <kd>1.0</kd>
  </gazebo>

  <!-- Right elbow -->
  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_shoulder_link"/>
    <child link="right_elbow_link"/>
    <origin xyz="0 0 -0.3"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="0.5" effort="30" velocity="3.0"/>
  </joint>

  <link name="right_elbow_link">
    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder radius="0.025" length="0.3"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder radius="0.025" length="0.3"/>
      </geometry>
    </collision>
  </link>

  <!-- Gazebo material for right elbow -->
  <gazebo reference="right_elbow_link">
    <material>Gazebo/Orange</material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
    <kp>1000000.0</kp>
    <kd>1.0</kd>
  </gazebo>

  <!-- Head -->
  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head_link"/>
    <origin xyz="0 0 1.1"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="2.0"/>
  </joint>

  <link name="head_link">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.05"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- Gazebo material for head -->
  <gazebo reference="head_link">
    <material>Gazebo/White</material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
    <kp>1000000.0</kp>
    <kd>1.0</kd>
  </gazebo>

  <!-- Gazebo controller interface -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/humanoid_robot</robotNamespace>
      <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
    </plugin>
  </gazebo>
</robot>
```

## Sensor Integration in Gazebo

Adding sensors to the humanoid robot model enables perception capabilities in simulation:

```xml
<!-- IMU Sensor -->
<gazebo reference="base_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <visualize>true</visualize>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
  </sensor>
</gazebo>

<!-- Camera Sensor -->
<gazebo reference="head_link">
  <sensor name="camera" type="camera">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <camera name="head_camera">
      <horizontal_fov>1.047</horizontal_fov> <!-- 60 degrees -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>10.0</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <frame_name>camera_optical_frame</frame_name>
      <min_depth>0.1</min_depth>
      <max_depth>10.0</max_depth>
    </plugin>
  </sensor>
</gazebo>

<!-- Depth Camera -->
<gazebo reference="head_link">
  <sensor name="depth_camera" type="depth">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <camera name="depth_head_camera">
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
      <clip>
        <near>0.1</near>
        <far>10.0</far>
      </clip>
    </camera>
    <plugin name="depth_camera_controller" filename="libgazebo_ros_openni_kinect.so">
      <baseline>0.2</baseline>
      <distortion_k1>0.0</distortion_k1>
      <distortion_k2>0.0</distortion_k2>
      <distortion_k3>0.0</distortion_k3>
      <distortion_t1>0.0</distortion_t1>
      <distortion_t2>0.0</distortion_t2>
      <point_cloud_cutoff>0.1</point_cloud_cutoff>
      <point_cloud_cutoff_max>10.0</point_cloud_cutoff_max>
      <frame_name>depth_camera_optical_frame</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

## Controller Configuration for Gazebo

To control the humanoid robot in simulation, we need proper controller configurations:

```yaml
# humanoid_robot_controllers.yaml
humanoid_robot:
  # Joint state controller
  joint_state_controller:
    type: joint_state_controller/JointStateController
    publish_rate: 50

  # Position controllers for each joint
  left_hip_position_controller:
    type: position_controllers/JointPositionController
    joint: left_hip_joint
    pid: {p: 100.0, i: 0.01, d: 10.0}

  right_hip_position_controller:
    type: position_controllers/JointPositionController
    joint: right_hip_joint
    pid: {p: 100.0, i: 0.01, d: 10.0}

  left_knee_position_controller:
    type: position_controllers/JointPositionController
    joint: left_knee_joint
    pid: {p: 100.0, i: 0.01, d: 10.0}

  right_knee_position_controller:
    type: position_controllers/JointPositionController
    joint: right_knee_joint
    pid: {p: 100.0, i: 0.01, d: 10.0}

  left_ankle_position_controller:
    type: position_controllers/JointPositionController
    joint: left_ankle_joint
    pid: {p: 50.0, i: 0.01, d: 5.0}

  right_ankle_position_controller:
    type: position_controllers/JointPositionController
    joint: right_ankle_joint
    pid: {p: 50.0, i: 0.01, d: 5.0}

  left_shoulder_position_controller:
    type: position_controllers/JointPositionController
    joint: left_shoulder_joint
    pid: {p: 50.0, i: 0.01, d: 5.0}

  right_shoulder_position_controller:
    type: position_controllers/JointPositionController
    joint: right_shoulder_joint
    pid: {p: 50.0, i: 0.01, d: 5.0}

  left_elbow_position_controller:
    type: position_controllers/JointPositionController
    joint: left_elbow_joint
    pid: {p: 30.0, i: 0.01, d: 3.0}

  right_elbow_position_controller:
    type: position_controllers/JointPositionController
    joint: right_elbow_joint
    pid: {p: 30.0, i: 0.01, d: 3.0}

  neck_position_controller:
    type: position_controllers/JointPositionController
    joint: neck_joint
    pid: {p: 10.0, i: 0.01, d: 1.0}
```

## Launch Files for Gazebo Simulation

Proper launch files orchestrate the entire simulation environment:

```python
# launch/humanoid_gazebo.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='humanoid_world',
        description='Choose one of the world files from `/humanoid_description/worlds`'
    )

    # Launch Gazebo with world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare('humanoid_description'),
                'worlds',
                LaunchConfiguration('world') + '.world'
            ])
        }.items()
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'humanoid_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0'
        ],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True}
        ]
    )

    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True}
        ]
    )

    # Load controllers
    controller_manager_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_controller',
            'left_hip_position_controller',
            'right_hip_position_controller',
            'left_knee_position_controller',
            'right_knee_position_controller',
            'left_ankle_position_controller',
            'right_ankle_position_controller',
            'left_shoulder_position_controller',
            'right_shoulder_position_controller',
            'left_elbow_position_controller',
            'right_elbow_position_controller',
            'neck_position_controller'
        ],
        parameters=[
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription([
        world_arg,
        gazebo,
        spawn_entity,
        robot_state_publisher,
        joint_state_publisher,
        controller_manager_spawner
    ])
```

## Physics Parameter Tuning

Optimizing physics parameters is crucial for realistic humanoid simulation:

```xml
<!-- Physics parameters for humanoid simulation -->
<physics type="ode">
  <!-- Time step - smaller for more accuracy but slower simulation -->
  <max_step_size>0.001</max_step_size>

  <!-- Real-time factor - 1.0 means simulation runs at real-time speed -->
  <real_time_factor>1.0</real_time_factor>

  <!-- Update rate - should be higher than control rate -->
  <real_time_update_rate>1000.0</real_time_update_rate>

  <gravity>0 0 -9.8</gravity>

  <ode>
    <!-- Solver parameters -->
    <solver>
      <type>quick</type>
      <iters>100</iters>
      <sor>1.3</sor>
    </solver>

    <!-- Constraints parameters -->
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

## Practical Exercise: Gazebo Simulation Setup

1. Create a complete humanoid robot model with proper URDF and Gazebo integration
2. Configure physics parameters for stable simulation
3. Add necessary sensors (IMU, camera, etc.) to the robot
4. Set up controller configurations for all joints
5. Create launch files to start the complete simulation environment
6. Test basic movement and sensor functionality in simulation

## Summary

This week, we've covered the fundamentals of Gazebo simulation for humanoid robotics:

- **World configuration** for creating realistic simulation environments
- **URDF enhancements** with Gazebo-specific elements for proper simulation
- **Sensor integration** to enable perception capabilities in simulation
- **Controller configuration** for realistic joint control
- **Launch file setup** to orchestrate the complete simulation environment
- **Physics parameter tuning** for stable and realistic simulation

Proper Gazebo simulation setup is essential for developing and testing humanoid robot behaviors before deployment to real hardware.