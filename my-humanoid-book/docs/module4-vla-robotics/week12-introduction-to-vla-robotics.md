---
sidebar_position: 1
---

# Week 12: Introduction to Vision-Language-Action Robotics

## Learning Objectives

By the end of this week, you will be able to:
- Understand the fundamentals of Vision-Language-Action (VLA) robotics
- Implement speech recognition for voice command processing
- Design LLM-based planning systems for action generation
- Integrate vision, language, and action components into a unified system
- Test voice-commanded robot control in simulation

## Introduction to VLA Robotics

Vision-Language-Action (VLA) robotics represents the next generation of intelligent robotic systems that can understand natural language commands and execute complex tasks in real-world environments. Unlike traditional robotics systems that require specific programming for each task, VLA systems leverage large language models (LLMs) to interpret human commands and generate appropriate action sequences.

### The VLA Paradigm

The VLA approach combines three critical capabilities:

1. **Vision**: Environmental perception and object recognition
2. **Language**: Natural language understanding and command interpretation
3. **Action**: Motor control and task execution

This integration enables robots to respond to high-level, natural language commands such as "Go to the kitchen and bring me the red cup" without requiring explicit programming for each possible scenario.

### Key Components of VLA Systems

A typical VLA system consists of:

- **Speech Recognition**: Converting voice commands to text
- **Language Understanding**: Interpreting commands using LLMs
- **Action Planning**: Generating sequences of robot actions
- **Action Execution**: Controlling robot motors and actuators
- **Perception Integration**: Using vision to inform actions

## Speech Recognition and Voice Interface

The voice interface is the primary human-robot interaction mechanism in VLA systems. It transforms spoken commands into actionable text that can be processed by the planning system.

### Voice Command Interface Implementation

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
import pyaudio
import wave
import openai
from openai import OpenAI
import os

class VoiceCommandInterface(Node):
    """
    Voice Command Interface node for humanoid robot
    Uses speech recognition to convert voice commands to text
    """
    def __init__(self):
        super().__init__('voice_command_interface')

        # Parameters
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('chunk_size', 1024)
        self.declare_parameter('recording_duration', 5.0)  # seconds
        self.declare_parameter('whisper_model', 'whisper-1')
        self.declare_parameter('openai_api_key', '')

        self.sample_rate = self.get_parameter('sample_rate').value
        self.chunk_size = self.get_parameter('chunk_size').value
        self.recording_duration = self.get_parameter('recording_duration').value
        self.whisper_model = self.get_parameter('whisper_model').value
        self.openai_api_key = self.get_parameter('openai_api_key').value

        # Set up OpenAI client
        if self.openai_api_key:
            self.client = OpenAI(api_key=self.openai_api_key)
        else:
            # Check for API key in environment variable
            api_key = os.getenv('OPENAI_API_KEY')
            if api_key:
                self.client = OpenAI(api_key=api_key)
            else:
                self.get_logger().warn('OpenAI API key not found. Using local STT.')
                self.client = None

        # Audio configuration
        self.format = pyaudio.paInt16
        self.channels = 1
        self.frames_per_buffer = self.chunk_size

        # Initialize PyAudio
        self.audio = pyaudio.PyAudio()

        # Create publishers
        self.voice_command_pub = self.create_publisher(String, '/humanoid_robot/voice_command', 10)
        self.interpretation_pub = self.create_publisher(String, '/humanoid_robot/interpreted_command', 10)

        # Create timer for voice recording
        self.voice_timer = self.create_timer(10.0, self.record_voice_command)

        self.get_logger().info('Voice Command Interface node initialized')

    def record_voice_command(self):
        """Record voice command from microphone"""
        self.get_logger().info('Recording voice command...')

        try:
            # Open audio stream
            stream = self.audio.open(
                format=self.format,
                channels=self.channels,
                rate=self.sample_rate,
                input=True,
                frames_per_buffer=self.frames_per_buffer
            )

            frames = []
            import time
            start_time = time.time()

            # Record for specified duration
            while time.time() - start_time < self.recording_duration:
                data = stream.read(self.chunk_size, exception_on_overflow=False)
                frames.append(data)

            # Stop and close stream
            stream.stop_stream()
            stream.close()

            # Save to temporary WAV file for processing
            temp_filename = '/tmp/voice_command.wav'
            wf = wave.open(temp_filename, 'wb')
            wf.setnchannels(self.channels)
            wf.setsampwidth(self.audio.get_sample_size(self.format))
            wf.setframerate(self.sample_rate)
            wf.writeframes(b''.join(frames))
            wf.close()

            # Process the audio file
            self.process_audio_file(temp_filename)

            # Clean up temporary file
            os.remove(temp_filename)

        except Exception as e:
            self.get_logger().error(f'Error recording voice command: {e}')

    def process_audio_file(self, filename):
        """Process audio file using Whisper API or local processing"""
        try:
            if self.client:
                # Use OpenAI Whisper API
                with open(filename, "rb") as audio_file:
                    transcript = self.client.audio.transcriptions.create(
                        model=self.whisper_model,
                        file=audio_file
                    )
                text = transcript.text
            else:
                # Fallback: Use local speech recognition
                text = self.process_with_local_stt(filename)

            if text.strip():
                self.get_logger().info(f'Voice command recognized: {text}')

                # Publish raw voice command
                cmd_msg = String()
                cmd_msg.data = text
                self.voice_command_pub.publish(cmd_msg)

                # Publish for interpretation
                self.interpretation_pub.publish(cmd_msg)
            else:
                self.get_logger().info('No speech detected in recording')

        except Exception as e:
            self.get_logger().error(f'Error processing audio file: {e}')

    def process_with_local_stt(self, filename):
        """Process audio using local speech recognition (placeholder)"""
        self.get_logger().warn('Using simulated local STT - replace with actual implementation')
        return "move forward two steps then turn left"  # Placeholder
```

### Local Speech Recognition Fallback

For environments without internet access or OpenAI API, a local speech recognition system provides offline capabilities:

```python
import speech_recognition as sr
from vosk import Model, KaldiRecognizer
import json
import os

class LocalVoiceRecognitionNode(Node):
    """
    Local Voice Recognition node for humanoid robot
    Provides offline speech recognition using Vosk or SpeechRecognition
    """
    def __init__(self):
        super().__init__('local_voice_recognition')

        # Parameters
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('chunk_size', 8192)
        self.declare_parameter('vosk_model_path', '/opt/vosk-model-small-en-us')
        self.declare_parameter('use_vosk', True)

        self.sample_rate = self.get_parameter('sample_rate').value
        self.chunk_size = self.get_parameter('chunk_size').value
        self.vosk_model_path = self.get_parameter('vosk_model_path').value
        self.use_vosk = self.get_parameter('use_vosk').value

        # Initialize speech recognition
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Adjust for ambient noise
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)

        # Try to initialize Vosk model
        self.vosk_model = None
        self.vosk_rec = None
        if self.use_vosk:
            try:
                if os.path.exists(self.vosk_model_path):
                    self.vosk_model = Model(self.vosk_model_path)
                    self.vosk_rec = KaldiRecognizer(self.vosk_model, self.sample_rate)
                    self.get_logger().info(f'Vosk model loaded from {self.vosk_model_path}')
                else:
                    self.get_logger().warn(f'Vosk model not found, falling back to SpeechRecognition')
                    self.use_vosk = False
            except Exception as e:
                self.get_logger().warn(f'Error loading Vosk model: {e}, falling back to SpeechRecognition')
                self.use_vosk = False

        # Create publishers
        self.voice_command_pub = self.create_publisher(String, '/humanoid_robot/voice_command', 10)
        self.interpretation_pub = self.create_publisher(String, '/humanoid_robot/interpreted_command', 10)

        # Create timer for voice recording
        self.voice_timer = self.create_timer(10.0, self.record_voice_command)

        self.get_logger().info('Local Voice Recognition node initialized')

    def record_voice_command(self):
        """Record voice command from microphone"""
        self.get_logger().info('Recording voice command (local)...')

        try:
            if self.use_vosk:
                # Use Vosk for recognition
                result = self.record_with_vosk()
            else:
                # Use SpeechRecognition library
                result = self.record_with_speech_recognition()

            if result and result.strip():
                self.get_logger().info(f'Voice command recognized: {result}')

                # Publish raw voice command
                cmd_msg = String()
                cmd_msg.data = result
                self.voice_command_pub.publish(cmd_msg)

                # Publish for interpretation
                self.interpretation_pub.publish(cmd_msg)
            else:
                self.get_logger().info('No speech detected in recording')

        except Exception as e:
            self.get_logger().error(f'Error recording voice command: {e}')

    def record_with_vosk(self):
        """Record and recognize using Vosk"""
        try:
            import pyaudio
            import time

            # Initialize PyAudio
            audio = pyaudio.PyAudio()

            # Open audio stream
            stream = audio.open(
                format=pyaudio.paInt16,
                channels=1,
                rate=self.sample_rate,
                input=True,
                frames_per_buffer=self.chunk_size
            )

            frames = []
            start_time = time.time()

            # Record for specified duration
            while time.time() - start_time < 5.0:  # recording_duration
                data = stream.read(self.chunk_size, exception_on_overflow=False)
                frames.append(data)

                # Process audio chunk with Vosk
                if self.vosk_rec.AcceptWaveform(data):
                    result = self.vosk_rec.Result()
                    result_json = json.loads(result)
                    if 'text' in result_json and result_json['text']:
                        stream.stop_stream()
                        stream.close()
                        audio.terminate()
                        return result_json['text']

            # Final result
            final_result = self.vosk_rec.FinalResult()
            final_json = json.loads(final_result)

            stream.stop_stream()
            stream.close()
            audio.terminate()

            return final_json.get('text', '')

        except Exception as e:
            self.get_logger().error(f'Error in Vosk recognition: {e}')
            return ""
```

## Language Understanding with LLMs

The language understanding component uses large language models to interpret commands and generate action plans. This is the core intelligence that enables the robot to understand high-level, natural language instructions.

### LLM-Based Planning System

```python
import openai
from openai import OpenAI
import json
import re
import time

class LLMPlanningNode(Node):
    """
    LLM-based Planning node for humanoid robot
    Interprets voice commands and generates action sequences
    """
    def __init__(self):
        super().__init__('llm_planning')

        # Parameters
        self.declare_parameter('openai_model', 'gpt-4-turbo')
        self.declare_parameter('openai_api_key', '')
        self.declare_parameter('max_retries', 3)

        self.openai_model = self.get_parameter('openai_model').value
        self.openai_api_key = self.get_parameter('openai_api_key').value
        self.max_retries = self.get_parameter('max_retries').value

        # Set up OpenAI client
        if self.openai_api_key:
            self.client = OpenAI(api_key=self.openai_api_key)
        else:
            api_key = os.getenv('OPENAI_API_KEY')
            if api_key:
                self.client = OpenAI(api_key=api_key)
            else:
                self.get_logger().warn('OpenAI API key not found. Planning will not work without it.')
                self.client = None

        # Create subscribers
        self.voice_command_sub = self.create_subscription(
            String, '/humanoid_robot/interpreted_command', self.voice_command_callback, 10)

        # Create publishers
        self.action_plan_pub = self.create_publisher(String, '/humanoid_robot/action_plan', 10)
        self.status_pub = self.create_publisher(String, '/humanoid_robot/planning_status', 10)

        if self.client:
            self.get_logger().info('LLM Planning node initialized with OpenAI client')
        else:
            self.get_logger().warn('LLM Planning node initialized without OpenAI client')

    def voice_command_callback(self, msg):
        """Process voice command and generate action plan"""
        command = msg.data
        self.get_logger().info(f'Received voice command: {command}')

        if not self.client:
            self.get_logger().error('OpenAI client not available for planning')
            return

        try:
            # Generate action plan using LLM
            action_plan = self.generate_action_plan(command)

            if action_plan:
                # Publish action plan
                plan_msg = String()
                plan_msg.data = json.dumps(action_plan)
                self.action_plan_pub.publish(plan_msg)

                self.get_logger().info(f'Generated action plan: {action_plan}')
            else:
                self.get_logger().warn('No action plan generated for command')

        except Exception as e:
            self.get_logger().error(f'Error processing voice command: {e}')

    def generate_action_plan(self, command: str) -> list:
        """
        Generate action plan from voice command using LLM
        """
        if not self.client:
            return []

        # Define the context and possible actions for the humanoid robot
        system_prompt = """
        You are an action planning assistant for a humanoid robot. Your job is to interpret natural language commands and generate a sequence of actions for the robot to execute.

        Available actions:
        1. NAVIGATE_TO_LOCATION: Move the robot to a specific location (x, y coordinates)
        2. MOVE_RELATIVE: Move the robot relative to its current position (forward, backward, left, right, distance in meters)
        3. TURN: Rotate the robot (left, right, angle in degrees)
        4. GRAB_OBJECT: Attempt to grab an object at the current location
        5. RELEASE_OBJECT: Release a held object
        6. SPEAK: Make the robot speak a message
        7. WAVE_ARM: Wave the robot's arm
        8. DETECT_OBJECTS: Use perception to detect objects in the environment
        9. WAIT: Wait for a specified duration in seconds

        The output should be a JSON array of action objects, each with 'action_type' and relevant parameters.

        Examples:
        - "Move forward 2 meters" -> [{"action_type": "MOVE_RELATIVE", "direction": "forward", "distance": 2.0}]
        - "Go to the kitchen" -> [{"action_type": "NAVIGATE_TO_LOCATION", "x": 3.5, "y": -1.2}]
        - "Turn left and move forward" -> [{"action_type": "TURN", "direction": "left", "angle": 90}, {"action_type": "MOVE_RELATIVE", "direction": "forward", "distance": 1.0}]
        - "Pick up the red ball" -> [{"action_type": "DETECT_OBJECTS"}, {"action_type": "GRAB_OBJECT", "object_type": "red ball"}]

        Respond with only the JSON array, no other text.
        """

        user_prompt = f"Command: {command}"

        for attempt in range(self.max_retries):
            try:
                response = self.client.chat.completions.create(
                    model=self.openai_model,
                    messages=[
                        {"role": "system", "content": system_prompt},
                        {"role": "user", "content": user_prompt}
                    ],
                    temperature=0.1,
                    max_tokens=500
                )

                response_text = response.choices[0].message.content.strip()

                # Extract JSON from response if it contains other text
                json_match = re.search(r'\[.*\]', response_text, re.DOTALL)
                if json_match:
                    json_str = json_match.group()
                    action_plan = json.loads(json_str)
                    return action_plan
                else:
                    # Try to parse the entire response as JSON
                    try:
                        action_plan = json.loads(response_text)
                        return action_plan
                    except json.JSONDecodeError:
                        self.get_logger().warn(f'Could not parse LLM response as JSON: {response_text}')
                        return []

            except Exception as e:
                self.get_logger().warn(f'LLM request failed (attempt {attempt + 1}): {e}')
                if attempt == self.max_retries - 1:
                    return []
                time.sleep(1)  # Wait before retry

        return []
```

## Practical Exercise: Setting Up VLA System

1. Install required dependencies (OpenAI API, speech recognition libraries)
2. Configure the voice command interface with appropriate audio devices
3. Test voice recognition with simple commands
4. Verify LLM planning generates appropriate action sequences

## Summary

This week, we've explored the fundamentals of Vision-Language-Action robotics and implemented the voice interface and LLM planning components. We've covered:

- Voice command recognition using OpenAI Whisper and local alternatives
- LLM-based action planning for interpreting natural language commands
- Integration of speech recognition and language understanding

Next week, we'll complete the VLA system by implementing the action execution system and testing the complete pipeline.