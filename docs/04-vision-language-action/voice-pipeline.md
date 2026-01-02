---
sidebar_label: "4.1 Voice Pipeline"
sidebar_position: 2
title: "Voice Pipeline: Speech Recognition with Whisper"
description: "Integrate OpenAI Whisper for voice command ingestion"
keywords: [whisper, speech recognition, asr, voice control, openai]
---

# 4.1 Voice Pipeline

> *"Voice is the most natural human interface—let your robot listen."*

---

## Learning Objectives

- Set up OpenAI Whisper for speech-to-text
- Implement real-time audio streaming
- Create a ROS 2 speech recognition node
- Handle wake words and command detection

---

## Whisper Overview

**OpenAI Whisper** is a state-of-the-art speech recognition model:

| Model | Parameters | Speed | Accuracy |
|-------|------------|-------|----------|
| tiny | 39M | Fastest | Good |
| base | 74M | Fast | Better |
| small | 244M | Medium | Great |
| medium | 769M | Slow | Excellent |
| large-v3 | 1.5B | Slowest | Best |

---

## Installation

```bash
# Install Whisper
pip install openai-whisper

# Install audio dependencies
pip install sounddevice soundfile numpy

# For GPU acceleration
pip install torch torchaudio --index-url https://download.pytorch.org/whl/cu118

# Download model (first run)
python -c "import whisper; whisper.load_model('base')"
```

---

## Basic Transcription

```python
import whisper
import sounddevice as sd
import numpy as np

# Load model
model = whisper.load_model("base")

def record_audio(duration: float = 5.0, 
                 sample_rate: int = 16000) -> np.ndarray:
    """Record audio from microphone"""
    print(f"🎤 Recording for {duration} seconds...")
    audio = sd.rec(
        int(duration * sample_rate),
        samplerate=sample_rate,
        channels=1,
        dtype=np.float32
    )
    sd.wait()
    print("✅ Recording complete")
    return audio.flatten()

def transcribe(audio: np.ndarray) -> str:
    """Transcribe audio using Whisper"""
    result = model.transcribe(audio, fp16=False)
    return result["text"].strip()

# Example usage
if __name__ == "__main__":
    audio = record_audio(duration=5.0)
    text = transcribe(audio)
    print(f"📝 Transcription: {text}")
```

---

## ROS 2 Speech Recognition Node

```python
#!/usr/bin/env python3
"""
Speech Recognition Node - Whisper-based voice command system
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import whisper
import sounddevice as sd
import numpy as np
import threading
import queue


class SpeechRecognitionNode(Node):
    def __init__(self):
        super().__init__('speech_recognition')
        
        # Parameters
        self.declare_parameter('model_size', 'base')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('chunk_duration', 3.0)
        self.declare_parameter('wake_word', 'robot')
        
        model_size = self.get_parameter('model_size').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.chunk_duration = self.get_parameter('chunk_duration').value
        self.wake_word = self.get_parameter('wake_word').value.lower()
        
        # Load Whisper model
        self.get_logger().info(f'Loading Whisper model: {model_size}')
        self.model = whisper.load_model(model_size)
        self.get_logger().info('Model loaded!')
        
        # Publishers
        self.text_pub = self.create_publisher(String, 'speech/text', 10)
        self.command_pub = self.create_publisher(String, 'speech/command', 10)
        
        # Audio queue
        self.audio_queue = queue.Queue()
        
        # State
        self.listening = False
        self.wake_word_detected = False
        
        # Start audio capture thread
        self.capture_thread = threading.Thread(target=self._audio_capture_loop)
        self.capture_thread.daemon = True
        self.capture_thread.start()
        
        # Processing timer
        self.timer = self.create_timer(0.1, self._process_audio)
        
        self.get_logger().info(f'Speech recognition ready. Wake word: "{self.wake_word}"')
    
    def _audio_capture_loop(self):
        """Continuously capture audio chunks"""
        chunk_samples = int(self.sample_rate * self.chunk_duration)
        
        while True:
            try:
                audio = sd.rec(
                    chunk_samples,
                    samplerate=self.sample_rate,
                    channels=1,
                    dtype=np.float32
                )
                sd.wait()
                self.audio_queue.put(audio.flatten())
            except Exception as e:
                self.get_logger().error(f'Audio capture error: {e}')
    
    def _process_audio(self):
        """Process audio from queue"""
        if self.audio_queue.empty():
            return
        
        audio = self.audio_queue.get()
        
        # Check if audio has speech (simple energy threshold)
        energy = np.mean(np.abs(audio))
        if energy < 0.01:
            return  # Silence, skip
        
        # Transcribe
        try:
            result = self.model.transcribe(
                audio,
                fp16=False,
                language='en'
            )
            text = result["text"].strip().lower()
            
            if not text:
                return
            
            self.get_logger().info(f'Heard: "{text}"')
            
            # Publish raw transcription
            msg = String()
            msg.data = text
            self.text_pub.publish(msg)
            
            # Check for wake word
            if self.wake_word in text:
                self.wake_word_detected = True
                # Extract command after wake word
                parts = text.split(self.wake_word, 1)
                if len(parts) > 1:
                    command = parts[1].strip()
                    if command:
                        self._publish_command(command)
            elif self.wake_word_detected:
                # Continue listening after wake word
                self._publish_command(text)
                
        except Exception as e:
            self.get_logger().error(f'Transcription error: {e}')
    
    def _publish_command(self, command: str):
        """Publish recognized command"""
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)
        self.get_logger().info(f'📢 Command: "{command}"')
        
        # Reset wake word state after command
        self.wake_word_detected = False


def main(args=None):
    rclpy.init(args=args)
    node = SpeechRecognitionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Wake Word Detection

For lower latency, use a dedicated wake word detector:

```python
import numpy as np
from collections import deque

class WakeWordDetector:
    """Simple energy-based wake word detection"""
    
    def __init__(self, wake_word: str, threshold: float = 0.7):
        self.wake_word = wake_word.lower()
        self.threshold = threshold
        self.recent_transcriptions = deque(maxlen=5)
    
    def check(self, transcription: str) -> tuple:
        """
        Check if wake word is detected
        Returns: (detected: bool, command: str or None)
        """
        text = transcription.lower()
        self.recent_transcriptions.append(text)
        
        # Check current transcription
        if self.wake_word in text:
            # Extract command after wake word
            idx = text.find(self.wake_word)
            command = text[idx + len(self.wake_word):].strip()
            return (True, command if command else None)
        
        return (False, None)
    
    def get_context(self) -> str:
        """Get recent context for better understanding"""
        return " ".join(self.recent_transcriptions)
```

---

## Streaming Transcription

For real-time applications:

```python
import whisper
import numpy as np
from typing import Generator
import sounddevice as sd

class StreamingTranscriber:
    """Real-time streaming transcription"""
    
    def __init__(self, model_size: str = "base"):
        self.model = whisper.load_model(model_size)
        self.sample_rate = 16000
        self.buffer_duration = 30  # Whisper's max context
        self.buffer = np.array([], dtype=np.float32)
    
    def add_audio(self, chunk: np.ndarray):
        """Add audio chunk to buffer"""
        self.buffer = np.concatenate([self.buffer, chunk])
        
        # Keep only last N seconds
        max_samples = self.buffer_duration * self.sample_rate
        if len(self.buffer) > max_samples:
            self.buffer = self.buffer[-max_samples:]
    
    def transcribe_buffer(self) -> str:
        """Transcribe current buffer"""
        if len(self.buffer) < self.sample_rate:  # Min 1 second
            return ""
        
        # Pad to 30 seconds if needed
        audio = whisper.pad_or_trim(self.buffer)
        
        # Get mel spectrogram
        mel = whisper.log_mel_spectrogram(audio).to(self.model.device)
        
        # Decode
        options = whisper.DecodingOptions(
            language="en",
            without_timestamps=True
        )
        result = whisper.decode(self.model, mel, options)
        
        return result.text
    
    def stream(self) -> Generator[str, None, None]:
        """Stream transcriptions"""
        chunk_size = int(self.sample_rate * 0.5)  # 500ms chunks
        
        def callback(indata, frames, time, status):
            self.add_audio(indata[:, 0])
        
        with sd.InputStream(
            samplerate=self.sample_rate,
            channels=1,
            callback=callback,
            blocksize=chunk_size
        ):
            while True:
                text = self.transcribe_buffer()
                if text:
                    yield text
```

---

## Exercises

### Exercise 4.1.1: Multi-Language Support
Extend the node to detect and transcribe multiple languages.

### Exercise 4.1.2: Custom Wake Words
Implement wake word detection using a small neural network.

### Exercise 4.1.3: Noise Reduction
Add audio preprocessing to filter background noise.

---

<div style={{textAlign: 'center', marginTop: '2rem'}}>

[← Back to Module 4](./index.md) | [Next: Cognitive Logic →](./cognitive-logic.md)

</div>
