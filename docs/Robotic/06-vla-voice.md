---
sidebar_position: 6
title: "Chapter 6: Vision-Language-Action"
sidebar_label: "6. VLA & Voice AI"
---

# Vision-Language-Action (VLA)

**Week 13**

## 🎯 Goal
This is where the magic happens. We connect the "Body" (ROS 2) to the "Mind" (LLMs). The goal is to allow the robot to understand natural language commands.

## 📚 Key Topics

### 1. OpenAI Whisper (Ears)
* Converting raw audio from the robot's microphone into text.
* Handling noise in a physical environment.

### 2. Cognitive Planning (Brain)
* Using an LLM (like GPT-4o or Llama 3) to break down a command.
* **Input:** "Clean the room."
* **LLM Output:** `[navigate_to_table(), find_trash(), pick_up_trash(), navigate_to_bin(), drop_trash()]`

### 3. VLA Models
* Models that process both **Images** and **Text** to output **Robot Actions** directly.

:::info Integration
We will write a Python node that listens to your voice, sends it to Whisper, and then publishes the text to a ROS 2 topic `/voice_command`.
:::

## 🛠️ Code Snippet: Voice to ROS 2
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import whisper

class VoiceNode(Node):
    def __init__(self):
        super().__init__('voice_ear')
        self.publisher_ = self.create_publisher(String, '/human_command', 10)
        self.model = whisper.load_model("base")

    def listen_and_publish(self):
        # Simulate audio capture
        result = self.model.transcribe("audio.mp3")
        msg = String()
        msg.data = result["text"]
        self.publisher_.publish(msg)