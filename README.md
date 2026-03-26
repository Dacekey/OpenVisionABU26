# OpenVisionABU26

**OpenVisionABU26** is a real-time robotics perception system for **ABU Robocon 2026**.  
It transforms a trained YOLO model into a deployable ROS 2 perception pipeline through a clear workflow:

**Model → Inference Engine → ROS 2 Node → Detection Topics → Downstream Robot Modules**

The project is designed as a **production-oriented perception baseline**: stable, modular, reproducible, and ready to integrate with planning, decision, and control systems.

---

## Overview

OpenVisionABU26 focuses on one core responsibility:

- detect target symbols in real time
- publish structured detection results
- support downstream robot logic through ROS 2 interfaces
- remain easy to launch, configure, and deploy

At this stage, the project emphasizes **system reliability and deployment readiness** over feature complexity.

---

## Workflow

The system follows a practical robotics perception workflow:

### 1. Model
A YOLO model is trained and exported to **ONNX** format.

### 2. Inference Engine
A custom **C++ YOLO inference engine** is used to run the model with:

- ONNX Runtime
- OpenCV
- CUDA support when available

### 3. ROS 2 Integration
The engine is wrapped into a **ROS 2 perception node** that subscribes to camera images, runs inference, and publishes structured outputs.

### 4. System Outputs
The node publishes:

- annotated debug image for visualization
- detection results for machine-to-machine integration

### 5. Downstream Integration
These outputs can be consumed by:

- planning
- decision
- navigation
- robot control
- future perception layers such as grouping or tracking

---

## System Pipeline

```text
Camera
  ↓
Image Topic
  ↓
YOLO Detection Node
  ↓
/yolo/detections
  ↓
Planning / Decision / Control
```

Debug flow:

```text
Camera
  ↓
YOLO Detection Node
  ↓
/yolo/image_annotated
```

---

## Core Features

- Real-time symbol detection using YOLO
- Custom C++ inference engine
- ROS 2 Jazzy integration
- Standardized detection publishing with `vision_msgs`
- Configurable runtime parameters via YAML
- One-command startup using launch files
- Docker-based deployment workflow
- Optional GPU acceleration
- Modular architecture for future expansion

---

## Repository Structure

```text
OpenVisionABU26/
├── src/
│   └── abu_yolo_ros/
│       ├── include/
│       ├── src/
│       ├── models/
│       ├── config/
│       ├── launch/
│       ├── CMakeLists.txt
│       └── package.xml
├── docs/
├── docker/
├── Dockerfile
└── README.md
```

---

## Main ROS 2 Interfaces

### Input
- `/image_raw`

### Outputs
- `/yolo/detections`
- `/yolo/image_annotated`

### Detection Message Type
- `vision_msgs/msg/Detection2DArray`

This keeps the perception layer easy to integrate with downstream ROS 2 systems without exposing internal engine details.

---

## Technology Stack

### Software
- Ubuntu 24.04
- ROS 2 Jazzy
- C++
- OpenCV
- ONNX Runtime
- Docker

### Hardware
- USB camera or laptop camera
- NVIDIA GPU recommended for accelerated inference

---

## Running Modes

OpenVisionABU26 can be used in two practical ways.

### Local Development
Best for rapid iteration and debugging.

Typical workflow:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 launch abu_yolo_ros yolo.launch.py
```

### Docker Deployment
Best for reproducibility, portability, and sharing with other users.

Typical workflow:

```bash
docker build -t openvision:latest .
docker run -it --rm \
  --device=/dev/video0 \
  --network host \
  openvision:latest bash
```

Optional GPU mode:

```bash
docker run -it --rm \
  --gpus all \
  --device=/dev/video0 \
  --network host \
  openvision:latest bash
```

---

## Quick Start

### Build
```bash
docker build -t openvision:latest .
```

### Run
```bash
docker run -it --rm \
  --device=/dev/video0 \
  --network host \
  openvision:latest bash
```

### Inside Container
```bash
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
ros2 launch abu_yolo_ros yolo.launch.py
```

---

## Verification

Basic checks after launch:

```bash
ros2 topic list
ros2 topic echo /yolo/detections
ros2 topic hz /image_raw
```

Expected high-level behavior:

- camera topic is active
- YOLO node launches successfully
- detections are published
- annotated image is available
- system remains stable during runtime

---

## Notes for Developers

The default configuration may need to be adjusted depending on the runtime environment.

For example:

- Docker paths typically use `/workspace/...`
- local ROS 2 workspace paths may use `/home/<user>/ros2_ws/...`

If running directly in a local workspace, update model paths in the YAML configuration accordingly.

---

## Current Status

**OpenVisionABU26** currently represents a stable **Perception System v1 baseline**.

Completed:

- custom YOLO inference engine
- ROS 2 perception node
- detection topic publishing
- YAML configuration
- launch system
- Dockerized runtime
- CPU and Docker validation
- GPU-enabled Docker host setup workflow

This baseline is already suitable for:

- internal team sharing
- technical demos
- deployment experiments
- downstream robotics integration

---

## Roadmap

Short-term:
- deployment polish
- performance benchmarking across hardware
- runtime monitoring improvements
- documentation refinement

Mid-term:
- robot-side deployment
- optimization for edge hardware
- stronger production validation

Future perception extensions:
- object grouping
- tracking
- KFS-level interpretation
- segmentation
- distance estimation

---

## Design Philosophy

OpenVisionABU26 is built with a simple principle:

> **Keep the perception module focused, stable, and easy to integrate.**

Rather than solving every robotics task inside one node, the project aims to provide a clean perception output layer that other modules can build upon.

---

## Intended Use

This project is intended for:

- ABU Robocon 2026 development
- robotics perception experiments
- ROS 2 integration workflows
- deployable computer vision pipelines

---

## License

This project is intended primarily for educational, research, and robotics competition purposes.

---

## Maintainer

**Project:** OpenVisionABU26  
**Context:** ABU Robocon 2026  
**Domain:** Robotics Perception / Vision Engineering

---

## Version

```text
Version: v1.0
Status: Stable perception baseline
Deployment readiness: Verified
```
