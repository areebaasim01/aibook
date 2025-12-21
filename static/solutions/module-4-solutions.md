# Module 4: Vision-Language-Action - Exercise Solutions

Solutions for all exercises in Module 4.

---

## Chapter 4.1: Voice Pipeline

### Exercise 4.1.1: Multi-Language Support

**Task:** Extend the Whisper integration to support multiple languages with automatic language detection.

```python
#!/usr/bin/env python3
"""Solution: Multi-Language Voice Recognition"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool
import whisper
import numpy as np
import sounddevice as sd
from dataclasses import dataclass
from typing import Optional, Tuple

@dataclass
class TranscriptionResult:
    text: str
    language: str
    confidence: float
    is_english: bool

class MultiLanguageWhisper(Node):
    def __init__(self):
        super().__init__('multi_language_whisper')
        
        # Parameters
        self.declare_parameter('model_size', 'base')
        self.declare_parameter('auto_translate', True)
        self.declare_parameter('target_language', 'en')
        
        model_size = self.get_parameter('model_size').value
        self.auto_translate = self.get_parameter('auto_translate').value
        self.target_lang = self.get_parameter('target_language').value
        
        # Load Whisper model
        self.get_logger().info(f'Loading Whisper {model_size} model...')
        self.model = whisper.load_model(model_size)
        self.get_logger().info('Model loaded!')
        
        # Supported languages
        self.supported_languages = {
            'en': 'English', 'es': 'Spanish', 'fr': 'French',
            'de': 'German', 'it': 'Italian', 'pt': 'Portuguese',
            'ru': 'Russian', 'ja': 'Japanese', 'ko': 'Korean',
            'zh': 'Chinese', 'ar': 'Arabic', 'hi': 'Hindi'
        }
        
        # Publishers
        self.transcription_pub = self.create_publisher(
            String, 'transcription', 10
        )
        self.language_pub = self.create_publisher(
            String, 'detected_language', 10
        )
        self.translated_pub = self.create_publisher(
            String, 'translated_text', 10
        )
        
        # Audio settings
        self.sample_rate = 16000
        self.chunk_duration = 3.0  # seconds
        
        # Start listening
        self.timer = self.create_timer(0.1, self.process_audio)
        self.audio_buffer = []
        
        self.get_logger().info('Multi-Language Whisper ready')
    
    def detect_language(self, audio: np.ndarray) -> Tuple[str, float]:
        """Detect spoken language from audio"""
        # Pad/trim to 30 seconds for language detection
        audio_padded = whisper.pad_or_trim(audio)
        
        # Make log-Mel spectrogram
        mel = whisper.log_mel_spectrogram(audio_padded).to(self.model.device)
        
        # Detect language
        _, probs = self.model.detect_language(mel)
        detected_lang = max(probs, key=probs.get)
        confidence = probs[detected_lang]
        
        return detected_lang, confidence
    
    def transcribe(self, audio: np.ndarray, language: Optional[str] = None) -> TranscriptionResult:
        """Transcribe audio with optional language hint"""
        
        # Detect language if not specified
        if language is None:
            language, lang_conf = self.detect_language(audio)
        else:
            lang_conf = 1.0
        
        # Transcription options
        options = {
            'language': language,
            'task': 'transcribe'
        }
        
        # Transcribe
        result = self.model.transcribe(audio, **options)
        
        transcription = TranscriptionResult(
            text=result['text'].strip(),
            language=language,
            confidence=lang_conf,
            is_english=(language == 'en')
        )
        
        return transcription
    
    def translate_to_english(self, audio: np.ndarray) -> str:
        """Translate non-English audio directly to English"""
        result = self.model.transcribe(audio, task='translate')
        return result['text'].strip()
    
    def process_audio(self):
        """Process audio buffer"""
        # In practice, collect audio from microphone
        # This is a simplified example
        
        if len(self.audio_buffer) < self.sample_rate * self.chunk_duration:
            return
        
        audio = np.array(self.audio_buffer[:int(self.sample_rate * self.chunk_duration)])
        self.audio_buffer = self.audio_buffer[int(self.sample_rate * self.chunk_duration):]
        
        # Transcribe
        result = self.transcribe(audio)
        
        # Publish transcription
        msg = String()
        msg.data = result.text
        self.transcription_pub.publish(msg)
        
        # Publish detected language
        lang_msg = String()
        lang_msg.data = f"{result.language} ({result.confidence:.2%})"
        self.language_pub.publish(lang_msg)
        
        self.get_logger().info(
            f'[{result.language.upper()}] {result.text}'
        )
        
        # Translate if needed
        if self.auto_translate and not result.is_english:
            translated = self.translate_to_english(audio)
            trans_msg = String()
            trans_msg.data = translated
            self.translated_pub.publish(trans_msg)
            self.get_logger().info(f'[EN] {translated}')
    
    def record_audio_callback(self, indata, frames, time, status):
        """Callback for audio recording"""
        if status:
            self.get_logger().warn(f'Audio status: {status}')
        self.audio_buffer.extend(indata[:, 0].tolist())

def main():
    rclpy.init()
    node = MultiLanguageWhisper()
    
    # Start audio stream
    with sd.InputStream(
        samplerate=node.sample_rate,
        channels=1,
        callback=node.record_audio_callback
    ):
        rclpy.spin(node)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### Exercise 4.1.2: Custom Wake Words

**Task:** Implement a wake word detection system that activates the voice pipeline.

```python
#!/usr/bin/env python3
"""Solution: Wake Word Detection with Porcupine"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import pvporcupine
import pyaudio
import struct
import numpy as np
from enum import Enum
from typing import List, Optional

class WakeWordState(Enum):
    LISTENING = "listening"
    ACTIVATED = "activated"
    PROCESSING = "processing"
    TIMEOUT = "timeout"

class WakeWordDetector(Node):
    def __init__(self):
        super().__init__('wake_word_detector')
        
        # Parameters
        self.declare_parameter('access_key', '')  # Picovoice access key
        self.declare_parameter('keywords', ['hey robot', 'robot'])
        self.declare_parameter('sensitivities', [0.7, 0.7])
        self.declare_parameter('activation_timeout', 10.0)  # seconds
        
        access_key = self.get_parameter('access_key').value
        keywords = self.get_parameter('keywords').value
        sensitivities = self.get_parameter('sensitivities').value
        self.timeout = self.get_parameter('activation_timeout').value
        
        # Initialize Porcupine (example with built-in keywords)
        try:
            self.porcupine = pvporcupine.create(
                access_key=access_key,
                keywords=['picovoice', 'computer']  # Built-in keywords
            )
            self.get_logger().info('Porcupine wake word engine initialized')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize Porcupine: {e}')
            self.get_logger().info('Using fallback volume-based activation')
            self.porcupine = None
        
        # Audio setup
        self.pa = pyaudio.PyAudio()
        self.audio_stream = self.pa.open(
            rate=self.porcupine.sample_rate if self.porcupine else 16000,
            channels=1,
            format=pyaudio.paInt16,
            input=True,
            frames_per_buffer=self.porcupine.frame_length if self.porcupine else 512
        )
        
        # State
        self.state = WakeWordState.LISTENING
        self.activation_time = None
        
        # Publishers
        self.activated_pub = self.create_publisher(Bool, 'wake_word_activated', 10)
        self.state_pub = self.create_publisher(String, 'wake_word_state', 10)
        
        # Timer for detection loop
        self.timer = self.create_timer(0.01, self.detect_wake_word)
        self.timeout_timer = None
        
        self.get_logger().info('Wake word detector ready. Say "Hey Robot"!')
    
    def detect_wake_word(self):
        """Main wake word detection loop"""
        if self.state == WakeWordState.ACTIVATED:
            # Already activated, skip detection
            return
        
        try:
            pcm = self.audio_stream.read(
                self.porcupine.frame_length if self.porcupine else 512,
                exception_on_overflow=False
            )
            pcm = struct.unpack_from(
                "h" * (self.porcupine.frame_length if self.porcupine else 512),
                pcm
            )
            
            if self.porcupine:
                keyword_index = self.porcupine.process(pcm)
                
                if keyword_index >= 0:
                    self.on_wake_word_detected(keyword_index)
            else:
                # Fallback: volume-based activation
                volume = np.abs(np.array(pcm)).mean()
                if volume > 3000:  # Loud enough
                    self.on_wake_word_detected(0)
                    
        except Exception as e:
            self.get_logger().error(f'Audio error: {e}')
    
    def on_wake_word_detected(self, keyword_index: int):
        """Handle wake word detection"""
        self.get_logger().info('🎤 Wake word detected!')
        
        self.state = WakeWordState.ACTIVATED
        self.activation_time = self.get_clock().now()
        
        # Publish activation
        msg = Bool()
        msg.data = True
        self.activated_pub.publish(msg)
        
        self.publish_state()
        
        # Set timeout
        self.timeout_timer = self.create_timer(
            self.timeout,
            self.on_activation_timeout
        )
    
    def on_activation_timeout(self):
        """Handle activation timeout"""
        self.get_logger().info('Activation timeout - returning to listening')
        
        self.state = WakeWordState.LISTENING
        self.publish_state()
        
        msg = Bool()
        msg.data = False
        self.activated_pub.publish(msg)
        
        if self.timeout_timer:
            self.timeout_timer.cancel()
            self.timeout_timer = None
    
    def deactivate(self):
        """Manually deactivate (e.g., after command processed)"""
        self.state = WakeWordState.LISTENING
        self.publish_state()
        
        if self.timeout_timer:
            self.timeout_timer.cancel()
            self.timeout_timer = None
    
    def publish_state(self):
        """Publish current state"""
        msg = String()
        msg.data = self.state.value
        self.state_pub.publish(msg)
    
    def destroy_node(self):
        """Cleanup"""
        if self.audio_stream:
            self.audio_stream.close()
        if self.pa:
            self.pa.terminate()
        if self.porcupine:
            self.porcupine.delete()
        super().destroy_node()

def main():
    rclpy.init()
    node = WakeWordDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### Exercise 4.1.3: Noise Reduction

**Task:** Implement audio preprocessing with noise reduction for better recognition in noisy environments.

```python
#!/usr/bin/env python3
"""Solution: Audio Noise Reduction Pipeline"""

import numpy as np
from scipy import signal
from scipy.io import wavfile
import noisereduce as nr
from dataclasses import dataclass
from typing import Optional
import webrtcvad

@dataclass
class AudioConfig:
    sample_rate: int = 16000
    frame_duration_ms: int = 30
    vad_aggressiveness: int = 2  # 0-3, higher = more aggressive
    noise_reduce_strength: float = 1.0

class AudioPreprocessor:
    """Audio preprocessing pipeline for voice recognition"""
    
    def __init__(self, config: Optional[AudioConfig] = None):
        self.config = config or AudioConfig()
        
        # Voice Activity Detection
        self.vad = webrtcvad.Vad(self.config.vad_aggressiveness)
        
        # Noise profile (learned from ambient noise)
        self.noise_profile = None
        self.noise_samples = []
        self.is_calibrated = False
    
    def calibrate_noise(self, ambient_audio: np.ndarray, duration_sec: float = 1.0):
        """Learn noise profile from ambient audio"""
        # Use first N seconds as noise reference
        samples = int(self.config.sample_rate * duration_sec)
        self.noise_profile = ambient_audio[:samples].astype(np.float32)
        self.is_calibrated = True
        print(f"Calibrated noise profile from {duration_sec}s of ambient audio")
    
    def reduce_noise(self, audio: np.ndarray) -> np.ndarray:
        """Apply noise reduction"""
        audio_float = audio.astype(np.float32)
        
        if self.is_calibrated and self.noise_profile is not None:
            # Spectral subtraction with learned profile
            reduced = nr.reduce_noise(
                y=audio_float,
                sr=self.config.sample_rate,
                y_noise=self.noise_profile,
                prop_decrease=self.config.noise_reduce_strength,
                stationary=True
            )
        else:
            # Auto noise reduction
            reduced = nr.reduce_noise(
                y=audio_float,
                sr=self.config.sample_rate,
                prop_decrease=self.config.noise_reduce_strength * 0.8  # Less aggressive
            )
        
        return reduced.astype(np.int16)
    
    def apply_bandpass_filter(self, 
                             audio: np.ndarray,
                             low_freq: float = 100,
                             high_freq: float = 4000) -> np.ndarray:
        """Apply bandpass filter for speech frequencies"""
        nyquist = self.config.sample_rate / 2
        low = low_freq / nyquist
        high = high_freq / nyquist
        
        # Design butterworth bandpass filter
        b, a = signal.butter(4, [low, high], btype='band')
        filtered = signal.filtfilt(b, a, audio.astype(np.float32))
        
        return filtered.astype(np.int16)
    
    def detect_voice_activity(self, audio: np.ndarray) -> np.ndarray:
        """Detect voice segments using VAD"""
        frame_samples = int(self.config.sample_rate * self.config.frame_duration_ms / 1000)
        
        # Ensure audio is int16
        if audio.dtype != np.int16:
            audio = (audio * 32767).astype(np.int16)
        
        voice_mask = np.zeros(len(audio), dtype=bool)
        
        for i in range(0, len(audio) - frame_samples, frame_samples):
            frame = audio[i:i + frame_samples]
            frame_bytes = frame.tobytes()
            
            try:
                is_speech = self.vad.is_speech(
                    frame_bytes,
                    self.config.sample_rate
                )
                if is_speech:
                    voice_mask[i:i + frame_samples] = True
            except:
                pass
        
        return voice_mask
    
    def extract_voice_segments(self, audio: np.ndarray) -> np.ndarray:
        """Extract only voice segments from audio"""
        voice_mask = self.detect_voice_activity(audio)
        
        # Add padding around voice segments
        padding = int(self.config.sample_rate * 0.1)  # 100ms padding
        padded_mask = np.copy(voice_mask)
        
        for i in range(len(voice_mask)):
            if voice_mask[i]:
                start = max(0, i - padding)
                end = min(len(padded_mask), i + padding)
                padded_mask[start:end] = True
        
        return audio[padded_mask]
    
    def normalize_volume(self, audio: np.ndarray, target_db: float = -20) -> np.ndarray:
        """Normalize audio volume"""
        audio_float = audio.astype(np.float32)
        
        # Calculate current RMS
        rms = np.sqrt(np.mean(audio_float ** 2))
        if rms == 0:
            return audio
        
        current_db = 20 * np.log10(rms / 32768)
        
        # Calculate gain needed
        gain_db = target_db - current_db
        gain = 10 ** (gain_db / 20)
        
        # Apply gain with clipping protection
        normalized = audio_float * gain
        normalized = np.clip(normalized, -32768, 32767)
        
        return normalized.astype(np.int16)
    
    def preprocess(self, audio: np.ndarray) -> np.ndarray:
        """Full preprocessing pipeline"""
        # 1. Bandpass filter
        filtered = self.apply_bandpass_filter(audio)
        
        # 2. Noise reduction
        denoised = self.reduce_noise(filtered)
        
        # 3. Volume normalization
        normalized = self.normalize_volume(denoised)
        
        # 4. Extract voice segments (optional)
        # voice_only = self.extract_voice_segments(normalized)
        
        return normalized

def main():
    # Example usage
    preprocessor = AudioPreprocessor()
    
    # Simulate noisy audio
    duration = 5.0
    sample_rate = 16000
    t = np.linspace(0, duration, int(sample_rate * duration))
    
    # Create test signal (300 Hz tone with noise)
    signal_clean = np.sin(2 * np.pi * 300 * t) * 10000
    noise = np.random.randn(len(t)) * 3000
    noisy_audio = (signal_clean + noise).astype(np.int16)
    
    print("Preprocessing audio...")
    
    # Calibrate with first 0.5 seconds
    preprocessor.calibrate_noise(noisy_audio, duration_sec=0.5)
    
    # Process
    clean_audio = preprocessor.preprocess(noisy_audio)
    
    # Calculate SNR improvement
    original_snr = 10 * np.log10(np.var(signal_clean) / np.var(noise))
    residual_noise = clean_audio.astype(float) - signal_clean
    improved_snr = 10 * np.log10(np.var(signal_clean) / np.var(residual_noise))
    
    print(f"Original SNR: {original_snr:.1f} dB")
    print(f"Improved SNR: {improved_snr:.1f} dB")
    print(f"SNR improvement: {improved_snr - original_snr:.1f} dB")

if __name__ == '__main__':
    main()
```

---

## Chapter 4.2: Cognitive Logic (LLMs)

### Exercise 4.2.1: Multi-Step Commands

**Task:** Parse complex multi-step commands into executable action sequences.

```python
#!/usr/bin/env python3
"""Solution: Multi-Step Command Parser"""

import json
from typing import List, Dict, Any, Optional
from dataclasses import dataclass, asdict
from enum import Enum
import openai
from anthropic import Anthropic

class ActionType(Enum):
    NAVIGATE = "navigate"
    PICK = "pick"
    PLACE = "place"
    LOOK = "look"
    SPEAK = "speak"
    WAIT = "wait"
    OPEN = "open"
    CLOSE = "close"

@dataclass
class RobotAction:
    action_type: ActionType
    target: str
    parameters: Dict[str, Any]
    preconditions: List[str]
    expected_duration: float  # seconds

@dataclass 
class ActionPlan:
    original_command: str
    actions: List[RobotAction]
    estimated_duration: float
    confidence: float

class MultiStepCommandParser:
    def __init__(self, llm_provider: str = "openai"):
        self.provider = llm_provider
        
        if llm_provider == "openai":
            self.client = openai.OpenAI()
        else:
            self.client = Anthropic()
        
        self.system_prompt = """You are a robot command parser. Convert natural language commands into structured action sequences.

For each command, output a JSON array of actions. Each action has:
- action_type: one of [navigate, pick, place, look, speak, wait, open, close]
- target: the object or location  
- parameters: action-specific parameters
- preconditions: what must be true before this action
- expected_duration: estimated seconds

Examples:
Command: "Go to the kitchen and get me a glass of water"
Actions:
[
  {"action_type": "navigate", "target": "kitchen", "parameters": {}, "preconditions": [], "expected_duration": 30},
  {"action_type": "look", "target": "glass", "parameters": {"search_area": "counter"}, "preconditions": ["at_kitchen"], "expected_duration": 5},
  {"action_type": "pick", "target": "glass", "parameters": {}, "preconditions": ["glass_visible"], "expected_duration": 3},
  {"action_type": "navigate", "target": "water_dispenser", "parameters": {}, "preconditions": ["holding_glass"], "expected_duration": 5},
  {"action_type": "open", "target": "water_dispenser", "parameters": {"action": "dispense"}, "preconditions": ["at_dispenser"], "expected_duration": 10},
  {"action_type": "navigate", "target": "user_location", "parameters": {}, "preconditions": ["glass_filled"], "expected_duration": 30},
  {"action_type": "speak", "target": "user", "parameters": {"message": "Here is your water"}, "preconditions": ["at_user"], "expected_duration": 2}
]

Only output valid JSON. Be thorough in preconditions."""

    def parse_command(self, command: str) -> ActionPlan:
        """Parse natural language command into action plan"""
        
        if self.provider == "openai":
            response = self.client.chat.completions.create(
                model="gpt-4-turbo-preview",
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": f'Parse this command: "{command}"'}
                ],
                temperature=0.1,
                response_format={"type": "json_object"}
            )
            result = json.loads(response.choices[0].message.content)
        else:
            response = self.client.messages.create(
                model="claude-3-opus-20240229",
                max_tokens=2000,
                messages=[
                    {"role": "user", "content": f'{self.system_prompt}\n\nParse this command: "{command}"\n\nOutput only JSON.'}
                ]
            )
            result = json.loads(response.content[0].text)
        
        # Parse actions
        actions = []
        total_duration = 0
        
        for action_data in result.get('actions', result if isinstance(result, list) else []):
            action = RobotAction(
                action_type=ActionType(action_data['action_type']),
                target=action_data['target'],
                parameters=action_data.get('parameters', {}),
                preconditions=action_data.get('preconditions', []),
                expected_duration=action_data.get('expected_duration', 5.0)
            )
            actions.append(action)
            total_duration += action.expected_duration
        
        return ActionPlan(
            original_command=command,
            actions=actions,
            estimated_duration=total_duration,
            confidence=0.9  # In practice, assess confidence
        )
    
    def validate_plan(self, plan: ActionPlan) -> List[str]:
        """Validate action plan for issues"""
        issues = []
        
        for i, action in enumerate(plan.actions):
            # Check preconditions can be met by previous actions
            for precond in action.preconditions:
                if not self._can_satisfy_precondition(precond, plan.actions[:i]):
                    issues.append(
                        f"Action {i} ({action.action_type.value}) has unmet precondition: {precond}"
                    )
            
            # Check for invalid action sequences
            if action.action_type == ActionType.PLACE:
                if not any(a.action_type == ActionType.PICK for a in plan.actions[:i]):
                    issues.append(f"Action {i}: PLACE without prior PICK")
        
        return issues
    
    def _can_satisfy_precondition(self, precond: str, prior_actions: List[RobotAction]) -> bool:
        """Check if precondition can be satisfied by prior actions"""
        # Simplified check - in practice, use a state machine
        precond_lower = precond.lower()
        
        for action in prior_actions:
            if 'at_' in precond_lower and action.action_type == ActionType.NAVIGATE:
                if action.target.lower() in precond_lower:
                    return True
            if 'holding' in precond_lower and action.action_type == ActionType.PICK:
                return True
            if 'visible' in precond_lower and action.action_type == ActionType.LOOK:
                return True
        
        return len(prior_actions) == 0 and 'at_' not in precond_lower

def main():
    parser = MultiStepCommandParser(llm_provider="openai")
    
    commands = [
        "Go to the living room and turn on the TV",
        "Get me the red book from the bedroom shelf and bring it here",
        "Clean up the kitchen - put the dishes in the dishwasher and wipe the counter"
    ]
    
    for cmd in commands:
        print(f"\n{'='*60}")
        print(f"Command: {cmd}")
        print('='*60)
        
        plan = parser.parse_command(cmd)
        
        print(f"\nAction Plan ({plan.estimated_duration:.0f}s estimated):")
        for i, action in enumerate(plan.actions):
            print(f"  {i+1}. {action.action_type.value.upper()} -> {action.target}")
            if action.parameters:
                print(f"      params: {action.parameters}")
            if action.preconditions:
                print(f"      requires: {action.preconditions}")
        
        issues = parser.validate_plan(plan)
        if issues:
            print(f"\n⚠️ Validation issues:")
            for issue in issues:
                print(f"  - {issue}")
        else:
            print(f"\n✅ Plan validated successfully")

if __name__ == '__main__':
    main()
```

---

### Exercise 4.2.2: Clarification Dialogue

**Task:** Implement a dialogue system that asks clarifying questions for ambiguous commands.

```python
#!/usr/bin/env python3
"""Solution: Clarification Dialogue System"""

import json
from typing import List, Dict, Optional, Tuple
from dataclasses import dataclass
from enum import Enum
import openai

class AmbiguityType(Enum):
    LOCATION = "location"
    OBJECT = "object"
    QUANTITY = "quantity"
    TIME = "time"
    PREFERENCE = "preference"
    CONFIRMATION = "confirmation"

@dataclass
class ClarificationQuestion:
    question: str
    ambiguity_type: AmbiguityType
    options: Optional[List[str]] = None
    required: bool = True

@dataclass
class DialogueState:
    original_command: str
    clarifications_needed: List[ClarificationQuestion]
    clarifications_received: Dict[str, str]
    is_complete: bool = False

class ClarificationDialogue:
    def __init__(self):
        self.client = openai.OpenAI()
        self.state: Optional[DialogueState] = None
        
        self.analysis_prompt = """Analyze this robot command for ambiguities.
Return JSON with:
- "is_clear": boolean - true if command is unambiguous
- "ambiguities": array of objects, each with:
  - "type": one of [location, object, quantity, time, preference, confirmation]
  - "description": what is ambiguous
  - "question": clarifying question to ask
  - "options": optional array of likely options
  - "required": boolean - is clarification critical?

Examples of ambiguous commands:
- "Get me the cup" -> which cup? where?
- "Put this away" -> put what? where?
- "Come here" -> where exactly is "here"?

Respond only with valid JSON."""

    def analyze_command(self, command: str) -> DialogueState:
        """Analyze command for ambiguities"""
        
        response = self.client.chat.completions.create(
            model="gpt-4-turbo-preview",
            messages=[
                {"role": "system", "content": self.analysis_prompt},
                {"role": "user", "content": f'Analyze: "{command}"'}
            ],
            temperature=0.1,
            response_format={"type": "json_object"}
        )
        
        result = json.loads(response.choices[0].message.content)
        
        clarifications = []
        for amb in result.get('ambiguities', []):
            clarifications.append(ClarificationQuestion(
                question=amb['question'],
                ambiguity_type=AmbiguityType(amb['type']),
                options=amb.get('options'),
                required=amb.get('required', True)
            ))
        
        self.state = DialogueState(
            original_command=command,
            clarifications_needed=clarifications,
            clarifications_received={},
            is_complete=result.get('is_clear', False)
        )
        
        return self.state
    
    def get_next_question(self) -> Optional[ClarificationQuestion]:
        """Get next clarification question"""
        if not self.state or self.state.is_complete:
            return None
        
        for clarification in self.state.clarifications_needed:
            if clarification.question not in self.state.clarifications_received:
                return clarification
        
        self.state.is_complete = True
        return None
    
    def provide_answer(self, question: str, answer: str):
        """Record answer to clarification question"""
        if self.state:
            self.state.clarifications_received[question] = answer
    
    def get_clarified_command(self) -> str:
        """Generate clarified command with all context"""
        if not self.state:
            return ""
        
        # Build context from clarifications
        context_parts = [self.state.original_command]
        
        for question, answer in self.state.clarifications_received.items():
            context_parts.append(f"- {question}: {answer}")
        
        # Use LLM to synthesize clear command
        response = self.client.chat.completions.create(
            model="gpt-4-turbo-preview",
            messages=[
                {"role": "system", "content": "Synthesize a clear, unambiguous robot command from the original command and clarifications."},
                {"role": "user", "content": "\n".join(context_parts)}
            ],
            temperature=0.1
        )
        
        return response.choices[0].message.content
    
    def generate_natural_response(self, stage: str) -> str:
        """Generate natural language responses"""
        responses = {
            "greeting": "I'll help you with that. Let me make sure I understand correctly.",
            "asking": "I have a quick question: ",
            "confirming": "Got it! Just to confirm: ",
            "ready": "Perfect, I understand now. I'll ",
            "apologize": "I'm sorry, I'm not sure I understood. "
        }
        return responses.get(stage, "")

class DialogueManager:
    """Manage full dialogue flow"""
    
    def __init__(self):
        self.dialogue = ClarificationDialogue()
    
    def process_command(self, command: str) -> str:
        """Process command with clarification dialogue"""
        state = self.dialogue.analyze_command(command)
        
        if state.is_complete:
            return f"✅ Command understood: {command}"
        
        responses = []
        responses.append(self.dialogue.generate_natural_response("greeting"))
        
        # Simulate dialogue
        for _ in range(len(state.clarifications_needed)):
            question = self.dialogue.get_next_question()
            if not question:
                break
            
            # Format question
            q_text = f"\n🤖: {question.question}"
            if question.options:
                q_text += f"\n   Options: {', '.join(question.options)}"
            responses.append(q_text)
            
            # Simulate user response (in practice, wait for actual input)
            if question.options:
                simulated_answer = question.options[0]
            else:
                simulated_answer = "[user would provide answer]"
            
            responses.append(f"👤: {simulated_answer}")
            self.dialogue.provide_answer(question.question, simulated_answer)
        
        clarified = self.dialogue.get_clarified_command()
        responses.append(f"\n✅ Clarified command: {clarified}")
        
        return "\n".join(responses)

def main():
    manager = DialogueManager()
    
    test_commands = [
        "Get me the cup",
        "Put this somewhere",
        "Clean that up",
        "Go there and wait"
    ]
    
    for cmd in test_commands:
        print(f"\n{'='*60}")
        print(f"Original command: \"{cmd}\"")
        print('='*60)
        
        result = manager.process_command(cmd)
        print(result)

if __name__ == '__main__':
    main()
```

---

### Exercise 4.2.3: Error Recovery

**Task:** Implement error handling and recovery strategies when actions fail.

```python
#!/usr/bin/env python3
"""Solution: Error Recovery System"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from typing import List, Dict, Optional, Callable
from dataclasses import dataclass, field
from enum import Enum
import json
import openai

class ErrorType(Enum):
    NAVIGATION_BLOCKED = "navigation_blocked"
    OBJECT_NOT_FOUND = "object_not_found"
    GRASP_FAILED = "grasp_failed"
    COLLISION = "collision"
    TIMEOUT = "timeout"
    HARDWARE = "hardware"
    UNKNOWN = "unknown"

class RecoveryStrategy(Enum):
    RETRY = "retry"
    ALTERNATIVE = "alternative"
    SKIP = "skip"
    ABORT = "abort"
    ASK_HUMAN = "ask_human"
    REPLAN = "replan"

@dataclass
class ErrorContext:
    error_type: ErrorType
    action_name: str
    details: str
    retry_count: int = 0
    max_retries: int = 3

@dataclass
class RecoveryPlan:
    strategy: RecoveryStrategy
    alternative_action: Optional[str] = None
    explanation: str = ""
    requires_confirmation: bool = False

class ErrorRecoverySystem(Node):
    def __init__(self):
        super().__init__('error_recovery')
        
        self.client = openai.OpenAI()
        
        # Recovery strategy handlers
        self.recovery_handlers: Dict[RecoveryStrategy, Callable] = {
            RecoveryStrategy.RETRY: self.handle_retry,
            RecoveryStrategy.ALTERNATIVE: self.handle_alternative,
            RecoveryStrategy.SKIP: self.handle_skip,
            RecoveryStrategy.ABORT: self.handle_abort,
            RecoveryStrategy.ASK_HUMAN: self.handle_ask_human,
            RecoveryStrategy.REPLAN: self.handle_replan,
        }
        
        # Error history for learning
        self.error_history: List[Dict] = []
        
        # Publishers
        self.recovery_pub = self.create_publisher(String, 'recovery_action', 10)
        self.human_request_pub = self.create_publisher(String, 'human_assistance_request', 10)
        
        # Known recovery patterns
        self.patterns = {
            ErrorType.NAVIGATION_BLOCKED: [
                RecoveryStrategy.ALTERNATIVE,
                RecoveryStrategy.RETRY,
                RecoveryStrategy.ASK_HUMAN
            ],
            ErrorType.OBJECT_NOT_FOUND: [
                RecoveryStrategy.RETRY,  # Look again
                RecoveryStrategy.ALTERNATIVE,  # Look elsewhere
                RecoveryStrategy.ASK_HUMAN
            ],
            ErrorType.GRASP_FAILED: [
                RecoveryStrategy.RETRY,
                RecoveryStrategy.ALTERNATIVE,  # Different grasp
                RecoveryStrategy.ASK_HUMAN
            ],
            ErrorType.COLLISION: [
                RecoveryStrategy.ABORT,  # Safety first
                RecoveryStrategy.ASK_HUMAN
            ],
            ErrorType.TIMEOUT: [
                RecoveryStrategy.RETRY,
                RecoveryStrategy.SKIP,
                RecoveryStrategy.ABORT
            ]
        }
        
        self.get_logger().info('Error Recovery System initialized')
    
    def analyze_error(self, context: ErrorContext) -> RecoveryPlan:
        """Analyze error and determine recovery strategy"""
        
        # Check retry limit
        if context.retry_count >= context.max_retries:
            if context.error_type != ErrorType.COLLISION:
                return RecoveryPlan(
                    strategy=RecoveryStrategy.ASK_HUMAN,
                    explanation=f"Failed {context.max_retries} times, requesting help"
                )
            else:
                return RecoveryPlan(
                    strategy=RecoveryStrategy.ABORT,
                    explanation="Safety error - aborting task"
                )
        
        # Use LLM for complex decisions
        recovery = self._llm_recovery_decision(context)
        
        return recovery
    
    def _llm_recovery_decision(self, context: ErrorContext) -> RecoveryPlan:
        """Use LLM to determine optimal recovery"""
        
        prompt = f"""A robot encountered an error. Determine the best recovery strategy.

Error Type: {context.error_type.value}
Failed Action: {context.action_name}
Details: {context.details}
Retry Count: {context.retry_count}/{context.max_retries}

Available strategies:
- RETRY: Try the same action again
- ALTERNATIVE: Try a different approach
- SKIP: Skip this action and continue
- ABORT: Stop the entire task
- ASK_HUMAN: Request human assistance
- REPLAN: Create new plan from current state

Consider:
1. Safety implications
2. Task importance
3. Likelihood of success
4. User experience

Return JSON with:
- "strategy": chosen strategy
- "alternative_action": if ALTERNATIVE, what to try
- "explanation": brief explanation for user
- "requires_confirmation": boolean"""

        response = self.client.chat.completions.create(
            model="gpt-4-turbo-preview",
            messages=[
                {"role": "system", "content": "You are a robot error recovery specialist."},
                {"role": "user", "content": prompt}
            ],
            temperature=0.1,
            response_format={"type": "json_object"}
        )
        
        result = json.loads(response.choices[0].message.content)
        
        return RecoveryPlan(
            strategy=RecoveryStrategy(result['strategy'].lower()),
            alternative_action=result.get('alternative_action'),
            explanation=result.get('explanation', ''),
            requires_confirmation=result.get('requires_confirmation', False)
        )
    
    def execute_recovery(self, context: ErrorContext, plan: RecoveryPlan) -> bool:
        """Execute the recovery plan"""
        handler = self.recovery_handlers.get(plan.strategy)
        
        if handler:
            return handler(context, plan)
        
        self.get_logger().error(f"No handler for strategy: {plan.strategy}")
        return False
    
    def handle_retry(self, context: ErrorContext, plan: RecoveryPlan) -> bool:
        """Handle retry strategy"""
        self.get_logger().info(f"Retrying action: {context.action_name}")
        
        msg = String()
        msg.data = json.dumps({
            "action": "retry",
            "original_action": context.action_name,
            "attempt": context.retry_count + 1
        })
        self.recovery_pub.publish(msg)
        
        return True
    
    def handle_alternative(self, context: ErrorContext, plan: RecoveryPlan) -> bool:
        """Handle alternative approach"""
        self.get_logger().info(f"Trying alternative: {plan.alternative_action}")
        
        msg = String()
        msg.data = json.dumps({
            "action": "alternative",
            "new_action": plan.alternative_action,
            "reason": plan.explanation
        })
        self.recovery_pub.publish(msg)
        
        return True
    
    def handle_skip(self, context: ErrorContext, plan: RecoveryPlan) -> bool:
        """Handle skip strategy"""
        self.get_logger().warn(f"Skipping action: {context.action_name}")
        
        msg = String()
        msg.data = json.dumps({
            "action": "skip",
            "skipped_action": context.action_name,
            "reason": plan.explanation
        })
        self.recovery_pub.publish(msg)
        
        return True
    
    def handle_abort(self, context: ErrorContext, plan: RecoveryPlan) -> bool:
        """Handle abort strategy"""
        self.get_logger().error(f"ABORTING task due to: {context.error_type.value}")
        
        msg = String()
        msg.data = json.dumps({
            "action": "abort",
            "reason": plan.explanation,
            "error_type": context.error_type.value
        })
        self.recovery_pub.publish(msg)
        
        return False  # Task not recovered
    
    def handle_ask_human(self, context: ErrorContext, plan: RecoveryPlan) -> bool:
        """Request human assistance"""
        self.get_logger().info("Requesting human assistance...")
        
        request = f"""🤖 I need your help!

I was trying to: {context.action_name}
But encountered: {context.error_type.value}
Details: {context.details}

{plan.explanation}

What should I do?"""
        
        msg = String()
        msg.data = request
        self.human_request_pub.publish(msg)
        
        return True  # Waiting for human
    
    def handle_replan(self, context: ErrorContext, plan: RecoveryPlan) -> bool:
        """Create new plan from current state"""
        self.get_logger().info("Generating new plan...")
        
        msg = String()
        msg.data = json.dumps({
            "action": "replan",
            "from_state": context.details,
            "explanation": plan.explanation
        })
        self.recovery_pub.publish(msg)
        
        return True

def main():
    rclpy.init()
    
    recovery = ErrorRecoverySystem()
    
    # Test error scenarios
    test_errors = [
        ErrorContext(
            ErrorType.OBJECT_NOT_FOUND,
            "pick_cup",
            "Cup not visible in expected location",
            retry_count=0
        ),
        ErrorContext(
            ErrorType.NAVIGATION_BLOCKED,
            "navigate_to_kitchen",
            "Path blocked by closed door",
            retry_count=1
        ),
        ErrorContext(
            ErrorType.COLLISION,
            "move_arm",
            "Unexpected contact detected",
            retry_count=0
        )
    ]
    
    for error in test_errors:
        print(f"\n{'='*60}")
        print(f"Error: {error.error_type.value}")
        print(f"Action: {error.action_name}")
        print(f"Details: {error.details}")
        print('='*60)
        
        plan = recovery.analyze_error(error)
        
        print(f"\nRecovery Strategy: {plan.strategy.value}")
        print(f"Explanation: {plan.explanation}")
        if plan.alternative_action:
            print(f"Alternative: {plan.alternative_action}")
        if plan.requires_confirmation:
            print("⚠️ Requires user confirmation")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

*Solutions verified with OpenAI GPT-4 and Anthropic Claude 3*
