# Vision-Language-Action (VLA) Robotics Module

This module implements a Vision-Language-Action system for the humanoid robot, enabling voice-commanded control and intelligent action planning.

## Overview

The VLA system consists of three main components:
1. **Vision**: Perception system for understanding the environment
2. **Language**: Natural language processing for command interpretation
3. **Action**: Motor control and execution system

## Components

### 1. Voice Command Interface
- Records voice commands using microphone
- Uses OpenAI Whisper API for speech recognition
- Provides local fallback speech recognition

### 2. LLM Planning Module
- Interprets voice commands using large language models
- Generates action sequences based on commands
- Supports various action types (navigation, manipulation, etc.)

### 3. Action Execution System
- Executes planned actions on the humanoid robot
- Controls motor systems and actuators
- Provides feedback on execution status

### 4. VLA Agent
- Main integration node connecting all components
- Coordinates vision, language, and action systems

## Installation

```bash
# Install required dependencies
pip install openai pyaudio speech-recognition vosk

# Build the ROS2 package
cd ~/humanoid-robotics-textbook/labs/module4-vla-robotics
colcon build --packages-select vla_agent
source install/setup.bash
```

## Usage

### Launch the complete VLA system:

```bash
# Set your OpenAI API key
export OPENAI_API_KEY="your-api-key-here"

# Launch the VLA system
ros2 launch vla_agent vla_system.launch.py
```

### Voice Commands

The system supports various voice commands such as:
- "Move forward 2 meters"
- "Turn left and go to the kitchen"
- "Pick up the red ball"
- "Wave your arm"
- "Stop"

## ROS2 Topics

- `/humanoid_robot/voice_command` - Raw voice commands
- `/humanoid_robot/interpreted_command` - Commands after LLM processing
- `/humanoid_robot/action_plan` - Generated action sequences
- `/humanoid_robot/execution_status` - Execution status updates
- `/humanoid_robot/vla_agent_status` - Agent status
- `/humanoid_robot/vla_agent_response` - Agent responses

## ROS2 Actions

- `navigate_to_pose` - Navigation goals for the robot

## Architecture

```
Voice Command → Voice Interface → LLM Planner → Action Executor → Robot
     ↓              ↓                ↓             ↓            ↓
   Microphone   STT/Whisper     NLP/LLM      Motor Ctrl   Physical Robot
```

The system uses ROS2 topics and actions for communication between components, ensuring loose coupling and modularity.