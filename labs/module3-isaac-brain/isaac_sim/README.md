# NVIDIA Isaac Sim Environment

This directory contains the setup and configuration for the NVIDIA Isaac Sim environment, used for synthetic data generation and AI training for the humanoid robot.

## Overview

NVIDIA Isaac Sim is a robotics simulator built on NVIDIA Omniverse that provides:
- High-fidelity physics simulation
- Synthetic data generation capabilities
- AI training environments
- Photorealistic rendering for perception tasks

## Prerequisites

- NVIDIA GPU with RTX or GTX 1080+ (with CUDA support)
- NVIDIA Omniverse installed
- Isaac Sim extension
- Docker (for containerized deployment)

## Setup Instructions

### 1. Install Isaac Sim

Isaac Sim can be installed through NVIDIA Omniverse Launcher:
1. Download and install NVIDIA Omniverse Launcher
2. Search for "Isaac Sim" in the extension marketplace
3. Install the Isaac Sim extension

### 2. Environment Configuration

The environment is configured using the following structure:

```
isaac_sim/
├── configs/           # Configuration files
├── assets/            # Robot and environment assets
├── scenarios/         # Simulation scenarios
├── scripts/           # Python scripts for automation
└── outputs/           # Generated synthetic data
```

### 3. Docker Setup (Optional)

For containerized deployment, use the provided Docker configuration:

```bash
# Build Isaac Sim container
docker build -t humanoid-isaac-sim -f Dockerfile.isaac .

# Run Isaac Sim container
docker run --gpus all -it --rm \
  --name humanoid-isaac-container \
  -v $(pwd)/isaac_sim:/workspace/isaac_sim \
  -p 8211:8211 \
  humanoid-isaac-sim
```

### 4. Synthetic Data Generation

The environment is configured for generating synthetic sensor data for:
- SLAM algorithms
- Object detection and recognition
- Navigation training
- Perception tasks

## Key Features

### Physics Simulation
- Accurate rigid body dynamics
- Collision detection
- Contact simulation
- Material properties

### Sensor Simulation
- RGB cameras with realistic distortion
- Depth sensors
- LIDAR with configurable parameters
- IMU simulation
- Force/torque sensors

### Synthetic Data Generation
- Photorealistic rendering
- Domain randomization
- Automatic annotation
- Multi-modal data fusion

## Integration with ROS

The Isaac Sim environment integrates with ROS through:
- Isaac ROS bridge packages
- ROS2 message interfaces
- TF tree management
- Sensor message formats

## Running the Simulation

To launch the Isaac Sim environment for the humanoid robot:

```bash
# Launch Isaac Sim with the humanoid robot scene
isaac-sim --exec scripts/launch_humanoid_scene.py --config configs/humanoid_sim_config.yaml
```

## Scenarios

The following scenarios are available for training and testing:

1. **SLAM Training**: Indoor environments with landmarks for mapping
2. **Object Detection**: Various objects and lighting conditions
3. **Navigation**: Obstacle avoidance and path planning
4. **Manipulation**: Object interaction and grasping

## Performance Considerations

- Use GPU acceleration for optimal performance
- Adjust rendering quality based on hardware capabilities
- Use domain randomization for robust model training
- Monitor memory usage during synthetic data generation

## Troubleshooting

Common issues and solutions:

- **GPU memory errors**: Reduce scene complexity or rendering resolution
- **Physics instability**: Adjust physics parameters in configuration
- **ROS connection issues**: Verify network settings and ROS master
- **Performance problems**: Check system requirements and GPU drivers

This environment provides a comprehensive platform for developing and testing AI capabilities for the humanoid robot in a controlled, repeatable setting.