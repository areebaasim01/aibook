#!/usr/bin/env python3
"""
Whisper ROS Node - Speech Recognition for Robot Control
Module 4: Vision-Language-Action

This node uses OpenAI Whisper for speech-to-text,
enabling voice control of the robot.

Usage:
    ros2 run my_robot_pkg whisper_ros_node

Requirements:
    pip install openai-whisper sounddevice numpy
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import whisper
import sounddevice as sd
import numpy as np
import threading
import queue
from typing import Optional


class WhisperROSNode(Node):
    """
    ROS 2 node for speech recognition using OpenAI Whisper.
    
    Features:
    - Continuous audio capture
    - Wake word detection
    - Command extraction
    - ROS 2 topic publishing
    """
    
    def __init__(self):
        super().__init__('whisper_ros')
        
        # Parameters
        self.declare_parameter('model_size', 'base')
        self.declare_parameter('wake_word', 'robot')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('chunk_duration', 3.0)
        self.declare_parameter('language', 'en')
        self.declare_parameter('energy_threshold', 0.01)
        
        self.model_size = self.get_parameter('model_size').value
        self.wake_word = self.get_parameter('wake_word').value.lower()
        self.sample_rate = self.get_parameter('sample_rate').value
        self.chunk_duration = self.get_parameter('chunk_duration').value
        self.language = self.get_parameter('language').value
        self.energy_threshold = self.get_parameter('energy_threshold').value
        
        # Publishers
        self.transcription_pub = self.create_publisher(
            String, 'speech/transcription', 10
        )
        self.command_pub = self.create_publisher(
            String, 'speech/command', 10
        )
        
        # Load Whisper model
        self.get_logger().info(f'🎤 Loading Whisper model: {self.model_size}')
        self.model = whisper.load_model(self.model_size)
        self.get_logger().info('✅ Whisper model loaded')
        
        # Audio queue
        self.audio_queue = queue.Queue()
        
        # State
        self.is_listening = True
        self.awaiting_command = False
        
        # Start audio capture thread
        self.audio_thread = threading.Thread(target=self._audio_loop)
        self.audio_thread.daemon = True
        self.audio_thread.start()
        
        # Processing timer
        self.process_timer = self.create_timer(0.1, self._process_audio)
        
        self.get_logger().info(f'🎙️ Whisper ROS ready. Wake word: "{self.wake_word}"')
    
    def _audio_loop(self):
        """Continuous audio capture loop (runs in background thread)"""
        chunk_samples = int(self.sample_rate * self.chunk_duration)
        
        while self.is_listening:
            try:
                # Record audio chunk
                audio = sd.rec(
                    chunk_samples,
                    samplerate=self.sample_rate,
                    channels=1,
                    dtype=np.float32
                )
                sd.wait()
                
                # Add to queue for processing
                self.audio_queue.put(audio.flatten())
                
            except Exception as e:
                self.get_logger().error(f'Audio capture error: {e}')
    
    def _process_audio(self):
        """Process audio from the queue (main ROS thread)"""
        if self.audio_queue.empty():
            return
        
        audio = self.audio_queue.get()
        
        # Check voice activity (simple energy check)
        energy = np.mean(np.abs(audio))
        if energy < self.energy_threshold:
            return  # Silence, skip processing
        
        # Transcribe with Whisper
        try:
            result = self.model.transcribe(
                audio,
                fp16=False,
                language=self.language
            )
            
            text = result['text'].strip()
            if not text:
                return
            
            # Publish raw transcription
            self._publish_transcription(text)
            
            # Check for wake word
            text_lower = text.lower()
            if self.wake_word in text_lower:
                self.awaiting_command = True
                # Extract command after wake word
                parts = text_lower.split(self.wake_word, 1)
                if len(parts) > 1 and parts[1].strip():
                    command = parts[1].strip()
                    self._publish_command(command)
                    self.awaiting_command = False
            elif self.awaiting_command:
                # Continue listening for command after wake word
                self._publish_command(text)
                self.awaiting_command = False
                
        except Exception as e:
            self.get_logger().error(f'Transcription error: {e}')
    
    def _publish_transcription(self, text: str):
        """Publish raw transcription"""
        msg = String()
        msg.data = text
        self.transcription_pub.publish(msg)
        self.get_logger().info(f'📝 Heard: "{text}"')
    
    def _publish_command(self, command: str):
        """Publish extracted command"""
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)
        self.get_logger().info(f'🎯 Command: "{command}"')
    
    def destroy_node(self):
        """Clean shutdown"""
        self.is_listening = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WhisperROSNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('👋 Shutting down Whisper ROS')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
