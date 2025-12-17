---
title: "Voice Command: Integrating OpenAI Whisper"
sidebar_label: "Voice Command"
---

# Voice Command: Integrating OpenAI Whisper

Natural language understanding is a critical component for robots to interact intuitively with humans. The first step in processing voice commands is accurate speech-to-text (STT) transcription. **OpenAI Whisper** is a robust and highly accurate STT model that excels at transcribing spoken language into text, even in noisy environments or with varied accents. Integrating Whisper into a robotics platform allows humans to issue commands verbally, paving the way for more natural human-robot interaction.

## Why OpenAI Whisper?

Before Whisper, many STT systems struggled with:
-   **Accuracy**: Often made errors with colloquialisms, jargon, or non-standard accents.
-   **Robustness**: Performance dropped significantly in noisy environments.
-   **Multilingual Support**: Limited to a few languages.

Whisper, trained on a massive dataset of diverse audio and text, overcomes many of these limitations, offering:
-   **High Accuracy**: State-of-the-art performance across a wide range of audio conditions.
-   **Multilingual Transcription**: Supports transcription in many languages.
-   **Language Identification**: Can automatically identify the spoken language.
-   **Robustness to Noise**: Performs well even with background noise.

## Integration Architecture in ROS 2

To integrate Whisper into a ROS 2 system, we typically follow these steps:

1.  **Audio Capture**: A ROS 2 node (e.g., a custom driver or a generic audio capture node) records audio from a microphone and publishes it as a stream of raw audio data (e.g., `audio_common_msgs/msg/AudioData`).
2.  **Whisper Node**: A dedicated ROS 2 node subscribes to this audio stream, processes it with the Whisper model, and publishes the transcribed text.
3.  **Command Interpretation**: A separate node subscribes to the transcribed text and interprets it as robot commands, potentially using an LLM for natural language understanding (as discussed in the next chapter).

## Setting up OpenAI Whisper

Whisper can be run locally on your robot's edge device (if it has sufficient compute, e.g., Jetson Orin with a GPU) or on a remote workstation/server. The Hugging Face `transformers` library provides an easy way to load and run Whisper models.

### Installation (Python)

```bash
pip install -q transformers torch torchaudio soundfile
pip install -q optimum # For ONNX Runtime optimizations
```

### Basic Usage (Python)

```python
import torch
from transformers import pipeline
import torchaudio
import soundfile as sf

# Load a pre-trained Whisper model
# Options: "tiny", "base", "small", "medium", "large", "large-v2", "large-v3"
# For edge devices, "tiny" or "base" might be more suitable.
pipe = pipeline("automatic-speech-recognition", model="openai/whisper-tiny")

# Assume you have an audio file
audio_file = "my_voice_command.wav" # Replace with actual audio input

# If you have audio data as a NumPy array or PyTorch tensor:
# input_audio, sampling_rate = torchaudio.load(audio_file)
# if sampling_rate != 16000: # Whisper expects 16kHz audio
#    resampler = torchaudio.transforms.Resample(orig_freq=sampling_rate, new_freq=16000)
#    input_audio = resampler(input_audio)
# transcription = pipe(input_audio.squeeze().numpy(), sampling_rate=16000)["text"]


# For file input:
transcription = pipe(audio_file)["text"]
print(f"Transcription: {transcription}")

# Example of saving a dummy audio file for testing
# dummy_audio = torch.randn(1, 16000) # 1 second of random noise at 16kHz
# sf.write("dummy_audio.wav", dummy_audio.squeeze().numpy(), 16000)
```

## ROS 2 Whisper Node (Conceptual)

A ROS 2 node for Whisper would typically:
1.  **Initialize `rclpy` and a `Node`**.
2.  **Create a Subscriber**: Listen to an audio topic (e.g., `/audio/raw`).
3.  **Implement a Callback**: When audio data arrives:
    -   Buffer the incoming audio.
    -   When a sufficient segment of audio is collected (e.g., 5 seconds, or silence is detected), pass it to the Whisper model.
    -   Process the audio with the Whisper pipeline.
4.  **Create a Publisher**: Publish the transcribed text (e.g., `std_msgs/msg/String`) to a topic (e.g., `/robot/voice_command_text`).

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# from audio_common_msgs.msg import AudioData # Assuming this message type is available
from transformers import pipeline
import numpy as np
import io
import soundfile as sf

class WhisperASRNode(Node):

    def __init__(self):
        super().__init__('whisper_asr_node')
        self.get_logger().info('Initializing Whisper ASR Node...')
        self.asr_pipeline = pipeline("automatic-speech-recognition", model="openai/whisper-base", device=0) # device=0 for GPU, -1 for CPU
        self.audio_subscriber = self.create_subscription(
            # AudioData, # Replace with actual audio message type
            String, # Using String for a simple demo for now
            '/audio/raw',
            self.audio_callback,
            10
        )
        self.text_publisher = self.create_publisher(String, '/robot/voice_command_text', 10)
        self.audio_buffer = []
        self.sample_rate = 16000 # Whisper's expected sample rate

        self.get_logger().info('Whisper ASR Node ready.')

    # This callback would process actual audio data
    def audio_callback(self, msg):
        # In a real scenario, msg.data would be raw audio bytes
        # For this demo, let's assume msg.data is a base64 encoded wav or a path
        # This is a simplification; actual audio handling is more complex
        
        # Example: if msg.data is raw audio bytes (numpy array or similar)
        # self.audio_buffer.extend(msg.data)
        # For now, let's simulate with a direct transcription
        self.get_logger().info(f"Received dummy audio data: {msg.data[:30]}...")
        
        try:
            # Simulate processing a received audio buffer or file
            # In a real system, you'd buffer audio and process segments
            # For demonstration, let's just transcribe the received string as if it's an audio path
            transcribed_text = self.asr_pipeline(msg.data)["text"] # Assuming msg.data is a path for now
            self.get_logger().info(f"Transcribed: {transcribed_text}")
            
            text_msg = String()
            text_msg.data = transcribed_text
            self.text_publisher.publish(text_msg)
        except Exception as e:
            self.get_logger().error(f"Error during ASR: {e}")

def main(args=None):
    rclpy.init(args=args)
    whisper_node = WhisperASRNode()
    rclpy.spin(whisper_node)
    whisper_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
*Note: The `audio_callback` is a simplification. Real audio processing would involve buffering audio chunks, handling sample rates, and potentially using VAD (Voice Activity Detection) to segment speech.*

By integrating OpenAI Whisper, robots can gain the ability to understand spoken commands, transforming human-robot interaction from a technical chore into a more natural and intuitive experience.
