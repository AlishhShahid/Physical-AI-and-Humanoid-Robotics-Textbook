# Setting Up Gazebo Simulation

## Prerequisites

Before setting up the Gazebo simulation for our humanoid robot, ensure you have:

- ROS 2 installed (Humble Hawksbill or later)
- Gazebo Garden or Fortress installed
- Completed Module 1 (ROS 2 Nervous System)
- Basic understanding of URDF (Unified Robot Description Format)

## Installing Gazebo

For Ubuntu users, install Gazebo Garden (recommended):

```bash
# Add the osrfoundation repository
sudo apt update && sudo apt install -y wget
wget https://packages.osrfoundation.org/gazebo.gpg -O /tmp/gazebo.gpg
sudo cp /tmp/gazebo.gpg /usr/share/keyrings/gazebo-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo.list > /dev/null

# Install Gazebo Garden
sudo apt update
sudo apt install gz-garden
```

For alternative installation methods, visit the official Gazebo documentation.

## Gazebo Simulation Components

Our Gazebo simulation for the humanoid robot includes several key components:

### World File

The world file defines the environment where our robot will operate:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="humanoid_world">
    <!-- Physics configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <!-- ODE solver configuration -->
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.0</sor>
        </solver>
        <constraints>
          <cfm>0.000001</cfm>
          <erp>0.2</erp>
        </constraints>
      </ode>
    </physics>

    <!-- Environment -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Optional obstacles -->
    <model name="box1">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>1 1 1</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>1 1 1</size></box>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### Robot Model with Gazebo Extensions

Our robot model extends the basic URDF with Gazebo-specific elements:

```xml
<!-- IMU sensor on the base link -->
<gazebo reference="base_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <visualize>true</visualize>
    <imu>
      <angular_velocity>
        <x><noise type="gaussian"><stddev>2e-4</stddev></noise></x>
        <y><noise type="gaussian"><stddev>2e-4</stddev></noise></y>
        <z><noise type="gaussian"><stddev>2e-4</stddev></noise></z>
      </angular_velocity>
      <linear_acceleration>
        <x><noise type="gaussian"><stddev>1.7e-2</stddev></noise></x>
        <y><noise type="gaussian"><stddev>1.7e-2</stddev></noise></y>
        <z><noise type="gaussian"><stddev>1.7e-2</stddev></noise></z>
      </linear_acceleration>
    </imu>
  </sensor>
</gazebo>

<!-- Joint state publisher plugin -->
<gazebo>
  <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
    <robotNamespace>/humanoid_robot</robotNamespace>
    <jointName>left_hip_joint, left_knee_joint, left_ankle_joint,
              right_hip_joint, right_knee_joint, right_ankle_joint,
              left_shoulder_joint, left_elbow_joint,
              right_shoulder_joint, right_elbow_joint</jointName>
    <updateRate>50</updateRate>
  </plugin>
</gazebo>
```

## Launching the Simulation

To launch the Gazebo simulation with your humanoid robot:

```bash
# Source your ROS 2 workspace
source ~/humanoid_ws/install/setup.bash

# Launch the simulation
ros2 launch gazebo_sim humanoid_simulation.launch.py
```

## Simulation Parameters

The simulation can be configured with various parameters:

- **Physics accuracy**: Balance between accuracy and performance
- **Update rate**: Frequency of physics calculations
- **Real-time factor**: Simulation speed relative to real time
- **Sensor noise**: Realistic sensor characteristics

## Testing the Simulation

Once the simulation is running:

1. Verify that the robot appears in the Gazebo environment
2. Check that joint states are being published: `ros2 topic echo /humanoid_robot/joint_states`
3. Confirm IMU data is available: `ros2 topic echo /humanoid_robot/imu/data`
4. Try sending commands to move the robot

## Troubleshooting

Common issues and solutions:

- **Robot falls through the ground**: Check mass/inertia values in URDF
- **Joints don't respond**: Verify transmission configurations
- **Simulation runs slowly**: Reduce physics update rate or simplify models
- **Sensors not publishing**: Check Gazebo plugin configurations

## Next Steps

In the next chapter, we'll explore how to configure and test the various sensors in our simulation environment.