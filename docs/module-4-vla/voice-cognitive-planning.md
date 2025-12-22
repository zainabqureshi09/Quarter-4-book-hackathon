---
sidebar_position: 2
id: voice-cognitive-planning
title: "Voice-to-Action & Cognitive Planning"
description: "Using OpenAI Whisper and LLMs to translate natural language into robot actions."
---

# Voice-to-Action & Cognitive Planning

## 1. Learning Objectives
By the end of this chapter, you will be able to:
- **Integrate** OpenAI Whisper for robust speech-to-text transcription.
- **Design** a "Cognitive Planner" using an LLM (like GPT-4) to decompose tasks.
- **Translate** natural language ("Clean the room") into structured ROS 2 actions.
- **Implement** a pipeline: Voice -> Text -> Plan -> Execution.

---

## 2. Core Explanation (Intermediate)

### Voice-to-Action (Whisper)
**OpenAI Whisper** is a state-of-the-art automatic speech recognition (ASR) system. It is robust to accents and background noise, making it ideal for real-world robotics.
- **Input**: Audio waveform from the robot's microphone.
- **Output**: Text string (e.g., "Go to the kitchen and find the apple").

### Cognitive Planning (LLMs)
Robots understand code (Move to X, Y, Z), not vague commands ("Clean up"). We use **Large Language Models (LLMs)** as a translation layer.
- **Prompt Engineering**: We give the LLM a list of available robot skills (e.g., `pick()`, `place()`, `navigate()`) and ask it to break down the user's request into a sequence of these skills.
- **Chain of Thought**: The LLM reasons: "To clean the room, I first need to look for trash. If I see a cup, I should pick it up..."

---

## 3. Beginner Simplification

**The Translator**:
Imagine the robot speaks "Robot Code" and you speak "English."
1.  **Whisper**: Listens to your voice and writes it down as text.
2.  **The Planner (LLM)**: Reads the text ("Make me a sandwich") and writes a recipe in Robot Code:
    1.  `walk_to(kitchen)`
    2.  `find(bread)`
    3.  `pick(bread)`
    4.  ...
3.  **The Robot**: Executes the recipe step-by-step.

---

## 4. Advanced Deep-Dive

### JSON Enforcement
To ensure the LLM outputs valid code that the robot can parse, we use **JSON Schema enforcement** or libraries like `guidance`.
- **Prompt**: "You are a robot planner. Output ONLY a JSON list of actions."
- **Response**: `[{"action": "navigate", "target": "kitchen"}, {"action": "pick", "object": "apple"}]`

### Closed-Loop Planning
A static plan might fail (e.g., the apple isn't in the kitchen). **Closed-Loop Planning** feeds the robot's feedback ("Object not found") back into the LLM, asking it to replan ("Okay, try searching the dining room instead").

---

## 5. Code Examples

### Voice-to-Plan Pipeline (Python)

```python
import openai
import whisper

# 1. Transcribe Audio
model = whisper.load_model("base")
result = model.transcribe("audio_command.wav")
user_text = result["text"] # "Get me a soda."

# 2. Cognitive Planning (LLM)
system_prompt = """
You are a helper robot. Available functions:
- navigate(location)
- pick(object)
- place(location)
Convert the user request into a list of function calls.
"""

response = openai.ChatCompletion.create(
    model="gpt-4",
    messages=[
        {"role": "system", "content": system_prompt},
        {"role": "user", "content": user_text}
    ]
)

plan = response.choices[0].message.content
print(plan)
# Output:
# navigate("kitchen")
# pick("soda")
# navigate("user")
# place("user_table")
```

---

## 6. Real-World Context
**Google PaLM-E**:
Google's PaLM-E model connects language directly to robot perception and control, allowing it to solve long-horizon tasks like "Bring me the rice chips from the drawer" by reasoning about the environment state visually.

---

## 7. Exercises
1.  **Whisper**: Record a command and transcribe it using the Whisper Python API.
2.  **Prompting**: Write a system prompt that forces an LLM to output valid JSON actions for a "Tidying Robot."
3.  **Integration**: Create a script that takes a text string and calls dummy Python functions (`print("Moving to...")`) based on the LLM's output.

---

## 8. Assessment Questions
1.  **Explain**: Why do we need an LLM for robotics instead of just hard-coding commands? (Flexibility to understand natural language and novel variations).
2.  **Debug**: The LLM outputs "Grab the thingy." Why does the robot fail? (The robot needs specific object IDs, not vague nouns. The prompt must enforce strict vocabulary).
3.  **Design**: How would you handle a user command that is dangerous ("Jump off the cliff")? (Add a "Safety Filter" layer before execution).
