#!/usr/bin/env python3
"""
Local Voice Recognition Module for VLA Robotics
Provides offline speech recognition capabilities
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import AudioData
import numpy as np
import pyaudio
import wave
import threading
import queue
import time
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
        self.declare_parameter('audio_device_index', -1)  # -1 for default
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('chunk_size', 8192)  # Larger chunks for Vosk
        self.declare_parameter('recording_duration', 5.0)  # seconds
        self.declare_parameter('vosk_model_path', '/opt/vosk-model-small-en-us')
        self.declare_parameter('use_vosk', True)  # Use Vosk if available, otherwise SpeechRecognition

        self.audio_device_index = self.get_parameter('audio_device_index').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.chunk_size = self.get_parameter('chunk_size').value
        self.recording_duration = self.get_parameter('recording_duration').value
        self.vosk_model_path = self.get_parameter('vosk_model_path').value
        self.use_vosk = self.get_parameter('use_vosk').value

        # Initialize speech recognition
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone(device_index=self.audio_device_index if self.audio_device_index >= 0 else None)

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
                    self.get_logger().warn(f'Vosk model not found at {self.vosk_model_path}, falling back to SpeechRecognition')
                    self.use_vosk = False
            except Exception as e:
                self.get_logger().warn(f'Error loading Vosk model: {e}, falling back to SpeechRecognition')
                self.use_vosk = False

        # Create publishers
        self.voice_command_pub = self.create_publisher(String, '/humanoid_robot/voice_command', 10)
        self.interpretation_pub = self.create_publisher(String, '/humanoid_robot/interpreted_command', 10)

        # Create timer for voice recording
        self.voice_timer = self.create_timer(10.0, self.record_voice_command)  # Record every 10 seconds

        # Audio buffer
        self.audio_queue = queue.Queue()

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
            while time.time() - start_time < self.recording_duration:
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

    def record_with_speech_recognition(self):
        """Record and recognize using SpeechRecognition library"""
        try:
            with self.microphone as source:
                self.get_logger().info('Listening...')
                # Listen for audio with timeout
                audio_data = self.recognizer.listen(source, timeout=self.recording_duration, phrase_time_limit=5)

                # Try different recognition services
                try:
                    # Use Google Web Speech API (online)
                    text = self.recognizer.recognize_google(audio_data)
                    return text
                except sr.RequestError:
                    # API was unreachable or unresponsive
                    self.get_logger().warn('Google Web Speech API unavailable')

                    try:
                        # Use Sphinx (offline)
                        text = self.recognizer.recognize_sphinx(audio_data)
                        return text
                    except sr.RequestError:
                        self.get_logger().warn('Sphinx recognition unavailable')
                        return ""
                    except sr.UnknownValueError:
                        self.get_logger().info('Sphinx could not understand audio')
                        return ""
                except sr.UnknownValueError:
                    self.get_logger().info('Google Web Speech could not understand audio')
                    return ""

        except sr.WaitTimeoutError:
            self.get_logger().info('Listening timed out')
            return ""
        except Exception as e:
            self.get_logger().error(f'Error in SpeechRecognition: {e}')
            return ""

    def destroy_node(self):
        """Clean up resources"""
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    voice_node = LocalVoiceRecognitionNode()

    try:
        rclpy.spin(voice_node)
    except KeyboardInterrupt:
        pass
    finally:
        voice_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()