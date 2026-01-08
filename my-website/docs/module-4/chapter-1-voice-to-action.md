---
title: Voice-to-Action Pipeline
sidebar_position: 2
tags: [voice-recognition, whisper, speech-to-text, robotics, command-interpreter, ros2-actions]
description: Learn how to implement voice-controlled robot systems using Whisper API for speech recognition and mapping natural language commands to ROS 2 actions.
---

# Voice-to-Action Pipeline

This chapter covers the fundamentals of implementing voice-controlled robot systems using the OpenAI Whisper API for speech recognition and mapping natural language commands to ROS 2 actions. This is the foundational capability that enables all other voice-controlled interactions in the Vision-Language-Action (VLA) framework.

## Overview of Voice-to-Action Systems

Voice-to-Action systems enable robots to understand and execute natural language commands through several key components:

- **Speech Recognition**: Converting spoken language to text
- **Intent Classification**: Understanding the user's intention from the text
- **Action Mapping**: Converting the intent to specific robot actions
- **Execution**: Sending commands to the robot's action servers

These systems form the foundation for more advanced cognitive planning and autonomous capabilities.

## Whisper API Integration for Speech Recognition

The OpenAI Whisper API provides state-of-the-art speech recognition capabilities that can be integrated into robotics applications. Here's how to implement it:

### Basic Whisper Integration

```python
import openai
import asyncio
from typing import Dict, Any

class WhisperVoiceRecognizer:
    def __init__(self, api_key: str):
        openai.api_key = api_key

    async def recognize_speech(self, audio_file_path: str) -> Dict[str, Any]:
        """
        Convert audio file to text using Whisper API
        """
        try:
            with open(audio_file_path, "rb") as audio_file:
                transcript = await openai.Audio.atranscribe(
                    model="whisper-1",
                    file=audio_file,
                    response_format="verbose_json",
                    timestamp_granularities=["segment"]
                )

            return {
                "text": transcript.text,
                "confidence": transcript.avg_logprob if hasattr(transcript, 'avg_logprob') else 0.8,  # Default confidence
                "language": transcript.language,
                "duration": transcript.duration
            }
        except Exception as e:
            print(f"Error in speech recognition: {str(e)}")
            return {
                "text": "",
                "confidence": 0.0,
                "language": "unknown",
                "duration": 0.0
            }
```

### Real-time Audio Processing

For real-time applications, you'll need to capture audio and process it in chunks:

```python
import pyaudio
import wave
import tempfile
import os
from datetime import datetime

class RealTimeVoiceProcessor:
    def __init__(self, recognizer: WhisperVoiceRecognizer):
        self.recognizer = recognizer
        self.chunk = 1024  # Record in chunks of 1024 samples
        self.format = pyaudio.paInt16  # 16 bits per sample
        self.channels = 1  # Mono
        self.rate = 44100  # Sampling rate in Hz

    def record_audio_clip(self, duration: int = 5) -> str:
        """
        Record audio for specified duration and save to temporary file
        """
        p = pyaudio.PyAudio()

        # Open stream
        stream = p.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )

        print(f"Recording for {duration} seconds...")
        frames = []

        for i in range(0, int(self.rate / self.chunk * duration)):
            data = stream.read(self.chunk)
            frames.append(data)

        print("Recording finished.")

        # Stop and close the stream
        stream.stop_stream()
        stream.close()
        p.terminate()

        # Save the recorded data as a WAV file
        temp_filename = os.path.join(tempfile.gettempdir(), f"recording_{datetime.now().strftime('%Y%m%d_%H%M%S')}.wav")

        wf = wave.open(temp_filename, 'wb')
        wf.setnchannels(self.channels)
        wf.setsampwidth(p.get_sample_size(self.format))
        wf.setframerate(self.rate)
        wf.writeframes(b''.join(frames))
        wf.close()

        return temp_filename
```

## Natural Language Processing for Command Interpretation

Once we have the text from speech recognition, we need to interpret the command and extract the intent:

### Intent Classification with LLM

```python
import openai
from typing import Dict, List, Optional

class CommandInterpreter:
    def __init__(self, api_key: str):
        openai.api_key = api_key
        self.valid_intents = ["NAVIGATE", "MANIPULATE", "PERCEIVE", "QUERY", "WAIT"]

    async def classify_intent(self, text: str, context: Dict[str, Any] = None) -> Dict[str, Any]:
        """
        Use GPT to classify the intent of a command
        """
        if not text.strip():
            return {
                "intent": "UNKNOWN",
                "parameters": {},
                "confidence": 0.0
            }

        # Prepare context information
        context_info = ""
        if context:
            if "robotPosition" in context:
                pos = context["robotPosition"]
                context_info += f"The robot is currently at position ({pos.get('x', 0)}, {pos.get('y', 0)}, {pos.get('z', 0)}). "

            if "environment" in context:
                context_info += f"The environment contains: {context['environment']}. "

        # Create a structured prompt for intent classification
        prompt = f"""
        You are a robot command interpreter. Given the following user command and context,
        classify the intent and extract relevant parameters.

        Context: {context_info}
        User Command: "{text}"

        Available intents: {', '.join(self.valid_intents)}

        Respond in the following JSON format:
        {{
            "intent": "...",
            "parameters": {{}},
            "confidence": 0.0-1.0
        }}

        For NAVIGATE intent, parameters should include direction, distance, or destination.
        For MANIPULATE intent, parameters should include object to manipulate and action.
        For PERCEIVE intent, parameters should include what to perceive.
        For QUERY intent, parameters should include what is being asked.
        For WAIT intent, parameters should include duration if specified.
        """

        try:
            response = await openai.ChatCompletion.acreate(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": "You are a precise robot command interpreter. Respond only with valid JSON."},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.1,  # Low temperature for more consistent responses
                max_tokens=200
            )

            import json
            result = json.loads(response.choices[0].message.content)

            # Validate the intent
            if result["intent"] not in self.valid_intents:
                result["intent"] = "UNKNOWN"
                result["confidence"] = 0.0

            return result

        except Exception as e:
            print(f"Error in intent classification: {str(e)}")
            return {
                "intent": "UNKNOWN",
                "parameters": {},
                "confidence": 0.0
            }
```

## Mapping Voice Commands to ROS 2 Actions

The final step is to map the interpreted commands to specific ROS 2 actions:

### ROS 2 Action Client for Navigation

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import math

class VoiceCommandExecutor(Node):
    def __init__(self):
        super().__init__('voice_command_executor')

        # Create action clients for different types of actions
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Publisher for status updates
        self.status_pub = self.create_publisher(String, 'voice_command_status', 10)

        self.get_logger().info("Voice Command Executor initialized")

    def execute_navigation_command(self, params: Dict[str, Any]) -> bool:
        """
        Execute navigation commands based on parameters
        """
        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Navigation action server not available")
            return False

        # Create goal message based on parameters
        goal_msg = NavigateToPose.Goal()

        # Set the target pose based on parameters
        if 'destination' in params:
            # In a real implementation, you'd have a way to translate destination names to coordinates
            # For now, we'll use some basic mapping
            destination = params['destination'].lower()

            if destination == 'kitchen':
                goal_msg.pose.pose.position.x = 2.0
                goal_msg.pose.pose.position.y = 1.0
            elif destination == 'living room':
                goal_msg.pose.pose.position.x = -1.0
                goal_msg.pose.pose.position.y = 0.5
            elif destination == 'bedroom':
                goal_msg.pose.pose.position.x = 0.0
                goal_msg.pose.pose.position.y = -2.0
            else:
                # If destination is described as coordinates or directions
                if 'x' in params and 'y' in params:
                    goal_msg.pose.pose.position.x = float(params['x'])
                    goal_msg.pose.pose.position.y = float(params['y'])
                else:
                    # Default to moving forward if no specific destination
                    goal_msg.pose.pose.position.x = 1.0
                    goal_msg.pose.pose.position.y = 0.0

        elif 'direction' in params and 'distance' in params:
            # Move in a specific direction for a certain distance
            direction = params['direction'].lower()
            distance = float(params['distance'])

            if direction in ['forward', 'ahead', 'north']:
                goal_msg.pose.pose.position.x = distance
            elif direction in ['backward', 'back', 'south']:
                goal_msg.pose.pose.position.x = -distance
            elif direction in ['left', 'west']:
                goal_msg.pose.pose.position.y = distance
            elif direction in ['right', 'east']:
                goal_msg.pose.pose.position.y = -distance

        # Set orientation to face forward
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        # Set header
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Send goal
        self._send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

        return True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')

        # Publish completion status
        status_msg = String()
        status_msg.data = "Navigation completed"
        self.status_pub.publish(status_msg)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback}')
```

## Practical Examples of Voice Command Implementations

### Complete Voice Command System

```python
import asyncio
import tempfile
import os
from typing import Dict, Any

class VoiceControlledRobot:
    def __init__(self, openai_api_key: str):
        self.whisper_recognizer = WhisperVoiceRecognizer(api_key=openai_api_key)
        self.command_interpreter = CommandInterpreter(api_key=openai_api_key)
        self.real_time_processor = RealTimeVoiceProcessor(self.whisper_recognizer)

        # Initialize ROS 2 context
        rclpy.init()
        self.executor_node = VoiceCommandExecutor()

        self.get_logger().info("Voice Controlled Robot System initialized")

    async def process_voice_command(self, audio_path: str = None) -> bool:
        """
        Complete pipeline: Audio -> Text -> Intent -> Action
        """
        # Step 1: If no audio path provided, record live audio
        if not audio_path:
            audio_path = self.real_time_processor.record_audio_clip(duration=5)

        # Step 2: Recognize speech
        recognition_result = await self.whisper_recognizer.recognize_speech(audio_path)

        if recognition_result["confidence"] < 0.5:  # Low confidence
            self.get_logger().warn(f"Low confidence recognition: {recognition_result['confidence']}")
            return False

        text = recognition_result["text"]
        self.get_logger().info(f"Recognized text: {text}")

        # Step 3: Interpret command
        context = {}  # In a real system, you'd get robot position, environment, etc.
        interpretation_result = await self.command_interpreter.classify_intent(text, context)

        if interpretation_result["confidence"] < 0.6:  # Low confidence in interpretation
            self.get_logger().warn(f"Low confidence interpretation: {interpretation_result['confidence']}")
            return False

        intent = interpretation_result["intent"]
        params = interpretation_result["parameters"]

        self.get_logger().info(f"Interpreted intent: {intent} with params: {params}")

        # Step 4: Execute action based on intent
        if intent == "NAVIGATE":
            success = self.executor_node.execute_navigation_command(params)
            return success
        elif intent == "MANIPULATE":
            # Implementation for manipulation would go here
            self.get_logger().info("Manipulation command received but not implemented in this example")
            return True
        elif intent == "PERCEIVE":
            # Implementation for perception would go here
            self.get_logger().info("Perception command received but not implemented in this example")
            return True
        else:
            self.get_logger().warn(f"Unknown or unsupported intent: {intent}")
            return False

    def get_logger(self):
        """Helper to access logger"""
        return self.executor_node.get_logger()

    def run(self):
        """
        Run the voice-controlled system
        """
        try:
            # In a real system, you might have a continuous listening loop
            # For this example, we'll just process one command
            loop = asyncio.get_event_loop()
            result = loop.run_until_complete(self.process_voice_command())

            if result:
                self.get_logger().info("Command executed successfully")
            else:
                self.get_logger().info("Command execution failed")

        except KeyboardInterrupt:
            self.get_logger().info("Shutting down...")
        finally:
            # Cleanup
            rclpy.shutdown()
```

## Configuration Examples for Whisper Integration

### Environment Configuration

```bash
# .env file for API keys and configuration
OPENAI_API_KEY=your_openai_api_key_here
WHISPER_MODEL=whisper-1
DEFAULT_CONFIDENCE_THRESHOLD=0.6
RECORDING_DURATION=5  # seconds
AUDIO_SAMPLE_RATE=44100
AUDIO_CHANNELS=1
```

### Python Configuration

```python
# config.py
class VoiceControlConfig:
    OPENAI_API_KEY = os.getenv('OPENAI_API_KEY')
    WHISPER_MODEL = os.getenv('WHISPER_MODEL', 'whisper-1')
    DEFAULT_CONFIDENCE_THRESHOLD = float(os.getenv('DEFAULT_CONFIDENCE_THRESHOLD', 0.6))
    RECORDING_DURATION = int(os.getenv('RECORDING_DURATION', 5))
    AUDIO_SAMPLE_RATE = int(os.getenv('AUDIO_SAMPLE_RATE', 44100))
    AUDIO_CHANNELS = int(os.getenv('AUDIO_CHANNELS', 1))

    # ROS 2 related configs
    NAVIGATION_SERVER_TIMEOUT = 5.0
    ACTION_CLIENT_TIMEOUT = 10.0
```

## Troubleshooting Voice Recognition Issues

### Common Problems and Solutions

#### 1. Poor Recognition Quality
- **Problem**: Whisper returns low-confidence results or incorrect text
- **Causes**:
  - Background noise in the recording
  - Low-quality microphone
  - Audio format issues
- **Solutions**:
  - Use noise cancellation techniques
  - Ensure microphone is positioned correctly
  - Verify audio format is supported (WAV, MP3, etc.)

#### 2. High Latency
- **Problem**: Long delays between speaking and command execution
- **Causes**:
  - Network latency to Whisper API
  - Large audio files taking time to upload
  - Slow intent classification
- **Solutions**:
  - Optimize audio file size before sending
  - Use local Whisper models for faster processing
  - Implement caching for common commands

#### 3. Incorrect Intent Classification
- **Problem**: LLM misinterprets the command intent
- **Causes**:
  - Ambiguous language in the command
  - Insufficient context provided
  - Temperature settings too high
- **Solutions**:
  - Use more specific language in prompts
  - Provide richer context to the LLM
  - Fine-tune temperature and other parameters

### Performance Optimization Tips

#### 1. Audio Processing Optimization
- **Preprocessing**: Apply noise reduction filters before sending to Whisper
- **Chunking**: Process audio in smaller chunks for real-time applications
- **Compression**: Use appropriate audio compression to reduce file sizes

#### 2. API Call Optimization
- **Caching**: Cache results for common commands
- **Batching**: Batch multiple API calls when possible
- **Timeouts**: Implement appropriate timeouts to prevent hanging

## Exercises for Voice-to-Action Implementation

### Exercise 1: Basic Voice Command Recognition
Implement a basic system that can recognize and execute simple movement commands.

**Requirements**:
- Set up Whisper API integration
- Create a command interpreter for basic movements (forward, backward, turn)
- Execute commands using ROS 2 navigation actions
- Validate recognition accuracy with different speakers

**Steps**:
1. Implement the WhisperVoiceRecognizer class
2. Create simple intent classification for movement commands
3. Connect to ROS 2 navigation system
4. Test with various voice commands

### Exercise 2: Context-Aware Command Processing
Extend the system to consider the robot's current state and environment.

**Requirements**:
- Incorporate robot position into command interpretation
- Consider environmental constraints in action planning
- Handle ambiguous commands with context
- Implement feedback mechanisms

**Implementation**:
- Extend the context parameter with robot state
- Modify intent classification to consider context
- Add validation checks before executing actions

### Exercise 3: Error Handling and Recovery
Implement robust error handling for voice recognition failures.

**Requirements**:
- Detect and handle low-confidence recognitions
- Implement retry mechanisms
- Provide feedback to user when commands fail
- Graceful degradation when system is uncertain

**Components**:
- Confidence threshold checking
- Retry logic with different parameters
- User feedback mechanisms
- Fallback command options

## Summary

The Voice-to-Action pipeline forms the foundation of the Vision-Language-Action framework, enabling robots to understand and respond to natural language commands. By integrating Whisper API for speech recognition, LLMs for intent classification, and ROS 2 for action execution, we create a system that can bridge the gap between human language and robotic action. This foundation enables more advanced cognitive planning and autonomous capabilities explored in later chapters.