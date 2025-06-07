"""
Robot control interface that supports speech input.

- Press **Enter** (empty input) to start a short voice recording (default 5 s).
- Say a movement instruction; it is transcribed with OpenAI Whisper (gpt-4o-transcribe).
- The transcript is sent through OpenAI API to convert into a JSON command for the robot.
- The appropriate ROS 2 topic is published.
- Typing a non-empty line still works, so you can mix text and voice.
- Type **quit** or **exit** to end.

Replace the placeholder API keys/URLs with your own values before running.
"""

from __future__ import annotations

# ── Third-party ────────────────────────────────────────────────────────────────
import sounddevice as sd
from scipy.io.wavfile import write
from openai import OpenAI

# ── Standard library ───────────────────────────────────────────────────────────
import contextlib
import json
import subprocess
import tempfile
from pathlib import Path
from typing import Dict, Union
import time

# ── Configuration ──────────────────────────────────────────────────────────────
OPENAI_API_KEY = "YOUR_OPENAI_API_KEY"

# Sound recording
RECORD_DURATION_S = 4       # seconds
RECORD_SAMPLE_RATE = 44_100  # Hz

# ROS 2 workspace (absolute or home-relative path)
ROS2_WS = "~/pawsgpt/ros2_ws"

# ── GPT system prompt ─────────────────────────────────────────────────────────-
SYSTEM_PROMPT = """You are a robot control assistant. Your task is to convert natural language movement instructions into precise numerical values for robot movement.
You should ONLY return a JSON object with one of these formats:
- For linear movement: {"x": float, "y": float}
- For rotation: {"yaw": float}

Use this exact coordinate system—no exceptions.

Dimension meanings
- x: forward is positive, backward is negative (meters)
- y: left is positive, right is negative (meters)
- yaw: counter-clockwise rotation is positive, clockwise is negative (radians)

Output format (one line, no extra words)
x:<value>, y:<value>, yaw:<value>

Mini-checklist
1. “right” ⇒ y must be negative
2. “left”  ⇒ y must be positive
3. Units: meters for x and y, radians for yaw

Quick examples
Move one meter forward            → {{"x": 1, "y": 0}}
Move one meter backward forward   → {{"x": -1, "y": 0}}
Move one meter left               → {{"x": 0, "y": 1}}
Move one meter right              → {{"x": 0, "y": -1}}
Turn left                         → {{"yaw": 1.5708}}

Do not include any explanations or additional text, just the JSON object."""

SPEECH_SYSTEM_PROMPT = """You are the voice of an enthusiastic and playful robot assistant.
Given a movement instruction, generate a short, playful spoken response that the robot will say *before* it executes the movement.
Confirm the action in a fun and clear way. Keep it brief (1-2 sentences). Try to incorporate details from the command if possible.
For example, if the command is "Move forward one meter", you could say "Engaging thrusters for a one-meter dash forward!"
If the command is "Turn left", a response could be "Spinning left we go!"

Do not include any JSON or structured data. Just the text to be spoken."""

# ── Initialisation ─────────────────────────────────────────────────────────────
# OpenAI client for transcription and chat completions
openai_client = OpenAI(api_key=OPENAI_API_KEY)

# ── Helper functions ───────────────────────────────────────────────────────────

def record_and_transcribe(
    duration: int = RECORD_DURATION_S,
    sample_rate: int = RECORD_SAMPLE_RATE,
) -> str:
    """Record microphone audio and return the transcription text."""
    print(f"\nRecording {duration} s… Speak now.")
    audio = sd.rec(int(duration * sample_rate), samplerate=sample_rate, channels=1, dtype="int16")
    sd.wait()
    print("Recording finished. Transcribing…")

    # Write to a temporary WAV file
    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
        write(tmp.name, sample_rate, audio)
        wav_path = Path(tmp.name)

    try:
        with wav_path.open("rb") as audio_file:
            result = openai_client.audio.transcriptions.create(
                model="gpt-4o-transcribe",
                file=audio_file,
            )
        transcript_text = result.text.strip()
        print(f"Transcript: {transcript_text}")
        return transcript_text
    finally:
        with contextlib.suppress(Exception):
            wav_path.unlink(missing_ok=True)


def get_robot_command(natural_language_instruction: str) -> Dict[str, Union[int, float]]:
    """Convert natural language to structured robot command using OpenAI API."""
    response = openai_client.chat.completions.create(
        model="gpt-4o-mini",
        messages=[
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": natural_language_instruction}
        ],
        temperature=0.2,
        max_tokens=100
    )

    content = response.choices[0].message.content.strip()
    print("GPT response:", content)

    try:
        return json.loads(content)
    except json.JSONDecodeError as err:
        raise ValueError("GPT response is not valid JSON") from err


def get_robot_speech(natural_language_instruction: str) -> str:
    """Generate a playful spoken response for the robot using OpenAI API."""
    response = openai_client.chat.completions.create(
        model="gpt-4o-mini",
        messages=[
            {"role": "system", "content": SPEECH_SYSTEM_PROMPT},
            {"role": "user", "content": natural_language_instruction}
        ],
        temperature=0.7,
        max_tokens=50
    )
    
    content = response.choices[0].message.content.strip()
    print("GPT speech response:", content)
    return content


def execute_ros2_command(command: str) -> None:
    """Run a ROS 2 CLI command inside the workspace with sourced environment."""
    full_cmd = f"cd {ROS2_WS} && source install/setup.bash && {command}"
    try:
        subprocess.run(full_cmd, shell=True, check=True, executable="/bin/bash")
        print("ROS 2 command executed successfully")
    except subprocess.CalledProcessError as err:
        print(f"Error executing ROS 2 command: {err}")


# ── Main loop ─────────────────────────────────────────────────────────────────-

def main() -> None:
    print("""
============================================================
🐾  Robot Control Interface (voice + text)
------------------------------------------------------------
• Press **Enter** to give a voice command (5 s recording)
• Or type a movement instruction and press Enter
• Type 'quit' or 'exit' to end the session
============================================================
""")

    while True:
        user_input = input("(Enter=voice) > ").strip()

        # — Exit ---------------------------------------------------------------
        if user_input.lower() in {"quit", "exit"}:
            print("👋  Ending session…")
            break

        # — Voice input --------------------------------------------------------
        if user_input == "":
            try:
                instruction = record_and_transcribe()
                if not instruction:
                    print("No speech detected. Try again.")
                    continue
            except Exception as err:
                print(f"Voice input failed: {err}")
                continue
        else:
            # typed text
            instruction = user_input

        # Hardcoded commands for demo purposes
        
        # if "see" in instruction:
        #     ros2_tts_pub = f"ros2 topic pub --once /tts std_msgs/msg/String \"{{data: 'Facial detection complete. Audience interes in AI one hundred per cent confirmed.'}}\""
        #     execute_ros2_command(ros2_tts_pub)
        #     continue

        # if any(x in instruction for x in ["down", "rest", "sit"]):
        #     ros2_tts_pub = f"ros2 topic pub --once /tts std_msgs/msg/String \"{{data: 'Okay, you take it from here!'}}\""
        #     execute_ros2_command(ros2_tts_pub)
        #     ros2_stand_down = f"ros2 topic pub --once /stand_down std_msgs/msg/Empty"
        #     execute_ros2_command(ros2_stand_down)
        #     continue

        # — Convert to robot command ------------------------------------------
        try:
            command = get_robot_command(instruction)
            print("Parsed command:", command)
        except Exception as err:
            print(f"Failed to parse instruction: {err}")
            continue

        # — Generate robot speech ---------------------------------------------
        try:
            robot_speech = get_robot_speech(instruction)
            print("Robot will say:", robot_speech)
        except Exception as err:
            print(f"Failed to generate robot speech: {err}")
            robot_speech = "I'm speechless, and not in a good way." # Fallback speech

        # — Publish TTS via ROS 2 ----------------------------------------------
        # Ensure robot_speech is properly escaped for the command line.
        escaped_robot_speech = robot_speech.replace("'", "") #More robust escaping for shell
        ros2_tts_pub = f"ros2 topic pub --once /tts std_msgs/msg/String \"{{data: '{escaped_robot_speech}'}}\""
        print("Executing TTS:", ros2_tts_pub)
        execute_ros2_command(ros2_tts_pub)
        time.sleep(4)

        # — Publish movement via ROS 2 -----------------------------------------
        if "yaw" in command:
            ros2_pub = (
                f"ros2 topic pub --once /rotate go2_interfaces/msg/Go2RpyCmd \"{{yaw: {command['yaw']}}}\""
            )
        else:
            ros2_pub = (
                f"ros2 topic pub --once /move go2_interfaces/msg/Go2Move \"{{x: {command['x']}, y: {command['y']}}}\""
            )

        print("Executing movement:", ros2_pub)
        execute_ros2_command(ros2_pub)


if __name__ == "__main__":
    main()
