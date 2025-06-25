# Embodied AI Car (EAI-demo)

This is a project to develop an Embodied AI car. The goal is to build an intelligent mobile platform through phased development, integrating multiple interaction methods including keyboard, vision, voice, and Large Language Models (LLM).

---

## Features

The project is divided into four core development stages, progressively endowing the car with more advanced intelligence:

-   [**Stage 1: Keyboard Control**](./stage1_keyboard_control/): Implements basic remote control of the car using a keyboard.
-   [**Stage 2: Vision Tracking**](./stage2_vision_tracking/): Uses the YOLO object detection algorithm to enable the car to automatically recognize and track specific objects.
-   [**Stage 3: Voice Control**](./stage3_voice_control/): Integrates ASR technology to control the car via voice commands.
-   [**Stage 4: Intelligent Control**](./stage4_intelligent_control/): Sends natural language commands to a Large Language Model, which then infers and generates specific actions for the car.

---

## Directory Structure

To keep the project organized and modular, the repository uses the following directory structure:

```
EAI-demo/
├── hardware/                  # Bill of materials, wiring diagrams, and assembly steps
├── stage1_keyboard_control/   # Code and instructions for Stage 1: Keyboard Control
├── stage2_vision_tracking/    # Code and instructions for Stage 2: Vision Tracking
├── stage3_voice_control/      # Code and instructions for Stage 3: Voice Control
├── stage4_intelligent_control/  # Code and instructions for Stage 4: Intelligent Control
├── images/                    # Image resources used in the project
├── LICENSE                    # MIT License
└── README.md                  # This document - Project Overview
```

---

## Getting Started

1.  First, visit the **`hardware`** folder and follow the instructions to assemble the car.
2.  Next, you can proceed through each stage folder in order, from `stage1` to `stage4`.
3.  Each stage folder contains a detailed `README.md` file with a complete guide to the environment setup, code deployment, and running steps for that stage.

## License

This project is licensed under the [MIT License](./LICENSE).
