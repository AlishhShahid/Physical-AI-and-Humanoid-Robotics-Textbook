#!/usr/bin/env python3
"""
Voice Command Interface for VLA Robotics
Integrates speech recognition to convert voice commands to text
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
import numpy as np
import pyaudio
import wave
import threading
import queue
import time
import openai
from openai import OpenAI
import os
import json

class VoiceCommandInterface(Node):
    """
    Voice Command Interface node for humanoid robot
    Uses speech recognition to convert voice commands to text
    """
    def __init__(self):
        super().__init__('voice_command_interface')

        # Parameters
        self.declare_parameter('audio_device_index', -1)  # -1 for default
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('chunk_size', 1024)
        self.declare_parameter('recording_duration', 5.0)  # seconds
        self.declare_parameter('whisper_model', 'whisper-1')
        self.declare_parameter('openai_api_key', '')

        self.audio_device_index = self.get_parameter('audio_device_index').value
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
                self.get_logger().warn('OpenAI API key not found. Voice recognition will use local processing if available.')
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
        self.voice_timer = self.create_timer(10.0, self.record_voice_command)  # Record every 10 seconds

        # Audio buffer
        self.audio_queue = queue.Queue()

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
                frames_per_buffer=self.frames_per_buffer,
                input_device_index=self.audio_device_index if self.audio_device_index >= 0 else None
            )

            frames = []
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
            import os
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
                # Fallback: Use local speech recognition if available
                # For now, we'll simulate this with a placeholder
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
        # This is a placeholder implementation
        # In a real implementation, you might use:
        # - SpeechRecognition library with Vosk, Sphinx, etc.
        # - Local Whisper model via transformers
        self.get_logger().warn('Using simulated local STT - replace with actual local implementation')
        return "move forward two steps then turn left"  # Placeholder

    def destroy_node(self):
        """Clean up resources"""
        self.audio.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    voice_node = VoiceCommandInterface()

    try:
        rclpy.spin(voice_node)
    except KeyboardInterrupt:
        pass
    finally:
        voice_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()