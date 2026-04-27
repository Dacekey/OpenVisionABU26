<h1 align="center">
🚀 OpenVisionABU26
</h1>

<p align="center">
ROS2 Perception System for ABU Robocon 2026 | FPTU Robotics Team
</p>

<p align="center">

<img src="https://img.shields.io/badge/Ubuntu-24.04-E95420?logo=ubuntu&logoColor=white">
<img src="https://img.shields.io/badge/ROS2-Jazzy-22314E?logo=ros&logoColor=white">
<img src="https://img.shields.io/badge/C++-17-00599C?logo=cplusplus&logoColor=white">
<img src="https://img.shields.io/badge/OpenCV-Computer%20Vision-5C3EE8?logo=opencv&logoColor=white">
<img src="https://img.shields.io/badge/ONNX-Runtime-yellow?logo=onnx&logoColor=white">
<img src="https://img.shields.io/badge/Ultralytics-YOLO-111111">
<img src="https://img.shields.io/badge/Docker-Deployment-2496ED?logo=docker&logoColor=white">
<img src="https://img.shields.io/badge/GPU-CUDA-76B900?logo=nvidia&logoColor=black">
<img src="https://img.shields.io/badge/Status-Stable-success">

</p>

**OpenVisionABU26** is a real-time robotics perception system for **ABU Robocon 2026**.

It transforms a trained YOLO model into a deployable ROS 2 perception pipeline:

```
Model → Inference Engine → ROS 2 Node → Detection Topics → Robot Modules
```

---

# 🧠 System Overview

OpenVisionABU26 focuses on one core responsibility:

- detect target symbols in real time
- publish structured detection results
- support downstream robot logic
- enable stable deployment using Docker

At this stage, the system emphasizes:

```
Reliability
Modularity
Deployment readiness
```

---

# 🔄 System Workflow

## End-to-End Pipeline

<!-- ![Tech Stack](docs/images/pipeline.png) -->
<p align="center">
  <img src="docs/images/pipeline.png" width="50%">
</p>

The perception pipeline follows a clear robotics workflow:

```
YOLO Model
    ↓
Inference Engine
    ↓
ROS 2 Node
    ↓
Detection Topics
    ↓
Robot Modules
```

---

# ⚙️ Technology Stack

<!-- ![Tech Stack](docs/images/tech_stack.png) -->

<p align="center">
  <img src="docs/images/tech_stack.png" width="50%">
</p>

Core technologies used in this system:

## Software

- Ubuntu 24.04
- ROS 2 Jazzy
- C++
- OpenCV
- ONNX Runtime
- Docker

## Hardware

- USB camera or laptop camera
- NVIDIA GPU (recommended)

---

# ✨ Core Features

- Real-time symbol detection using YOLO
- Custom C++ inference engine
- ROS 2 Jazzy integration
- Structured detection publishing
- YAML-based configuration
- Launch file system
- Docker deployment
- Optional GPU acceleration
- Modular system architecture

---

# 📦 Deployment Modes

OpenVisionABU26 supports two primary execution modes.

---

## 🧪 Local Development

Best for rapid iteration and debugging.

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 launch abu_yolo_ros yolo.launch.py
```

---

## 🐳 Docker Deployment

Best for reproducibility and sharing.

```bash
docker build -t openvision:latest .
```

Run container:

```bash
docker run -it --rm \
  --device=/dev/video0 \
  --network host \
  openvision:latest bash
```

---

## ⚡ Optional GPU Mode

```bash
docker run -it --rm \
  --gpus all \
  --device=/dev/video0 \
  --network host \
  openvision:latest bash
```

---

# 🚀 Quick Start

## Build

```bash
docker build -t openvision:latest .
```

## Run

```bash
docker run -it --rm \
  --device=/dev/video0 \
  --network host \
  openvision:latest bash
```

Inside container:

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
ros2 launch abu_yolo_ros yolo.launch.py
```

---

# 🔍 Verification

After launching the system:

```bash
ros2 topic list
ros2 topic echo /yolo/detections
ros2 topic hz /image_raw
```

Expected behavior:

- camera topic is active
- YOLO node launches successfully
- detections are published
- `/yolo/kfs_instances` is available as `abu_yolo_ros/msg/KfsInstanceArray` when `kfs_instance_aggregation.publish_instances=true`
- `/yolo/kfs_instances_localized` is available as `abu_yolo_ros/msg/LocalizedKfsInstanceArray` when `kfs_3d_localizer.enabled=true`
- `/yolo/kfs_instances/image_annotated` is available when `kfs_instance_aggregation.publish_debug_image=true`
- the KFS debug image can also draw the aggregation ROI when `kfs_instance_aggregation.draw_roi=true`
- system runs stably

---

# 🗂 Repository Structure

```
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
│
├── docs/
│   ├── images/
│   │
│   └── Guide_Documents
│
├── docker/
├── Dockerfile
└── README.md
```

---

# 🧩 System Interfaces

## Input

```
/image_raw
```

## Outputs

```
/yolo/detections
/yolo/kfs_instances
/yolo/kfs_instances_localized
/yolo/kfs_instances_stabilized
/yolo/image_annotated
/yolo/kfs_instances/image_annotated
```

`/yolo/detections` remains the symbol-level output.

`/yolo/kfs_instances` is the KFS-level structured runtime output and uses the message type `abu_yolo_ros/msg/KfsInstanceArray`.

The KFS instance publisher now evaluates team color at the instance level using the final KFS crop, with `refined_bbox` as the primary source and `expanded_bbox` as fallback when needed.

Decision labels in the vision pipeline are legality-oriented, not immediate robot action commands:

- `legal`: valid KFS target candidate
- `illegal`: invalid or rule-forbidden KFS target candidate
- `unknown`: insufficient evidence

Downstream planning can later decide whether to collect, skip, or route around a localized legal KFS candidate.

`/yolo/kfs_instances_localized` is the first OpenVision-v3 KFS 3D localization output and uses the message type `abu_yolo_ros/msg/LocalizedKfsInstanceArray`.

The standalone localizer uses `bottom_center` as the primary projection point and `center` as fallback, supports `none`, `pinhole`, and `fisheye` point undistortion modes, transforms `ray_camera -> ray_robot`, and intersects that ray with a plane height selection stage.

Current plane-height design:

- `fixed` is the only active stable mode right now.
- valid Meihua heights are `0`, `200`, `400`, and `600` mm.
- `instance_hint` is intentionally postponed because visual color/type-based height inference may be unreliable.
- `block_map` is the preferred future mode once a BlockLocalizer / board-map provider exists.
- until that provider exists, `block_map` still falls back to fixed plane height.

`/yolo/kfs_instances_stabilized` reuses the same `abu_yolo_ros/msg/LocalizedKfsInstanceArray` contract and adds lightweight perception-level temporal smoothing after 3D localization.

The stabilizer uses a small constant-velocity Kalman filter with conservative gating. It smooths localized KFS positions for downstream consumers, but it is not strategic KFS tracking, not closest-KFS selection, and not collection-priority memory. Visual servoing is also future work and is not implemented here.

`/yolo/kfs_instances/image_annotated` remains a debug-only visualization topic for final KFS instance aggregation boxes. It can also draw the aggregation ROI overlay, and that ROI should match the actual filtering region used by the runtime aggregator. It does not change the existing `/yolo/detections` or `/yolo/image_annotated` outputs.

To inspect it at runtime:

```bash
ros2 topic list | grep kfs
rqt_image_view
```

In `rqt_image_view`, select `/yolo/kfs_instances/image_annotated`. The topic is published only when `kfs_instance_aggregation.enabled=true`, `kfs_instance_aggregation.debug_instances=true`, and `kfs_instance_aggregation.publish_debug_image=true`.

## Message Type

```
vision_msgs/msg/Detection2DArray
```

---

# ⚠️ Notes for Developers

Paths may differ depending on runtime environment.

Example:

```
Docker:
    /workspace/...

Local:
    /home/<user>/ros2_ws/...
```

If running locally, update model paths in:

```
config.yaml
```

---

# 📊 Current Status

OpenVisionABU26 represents:

```
Perception System v1
Stable baseline
Deployment-ready
```

Completed components:

- YOLO inference engine
- ROS 2 node
- detection publishing
- launch system
- Docker deployment
- GPU runtime support

---

# 🗺 Roadmap

## Short-term

- deployment polish
- performance benchmarking
- runtime monitoring
- documentation refinement

## Mid-term

- robot-side deployment
- edge hardware optimization
- production validation

## Future Extensions

- object grouping
- tracking
- segmentation
- distance estimation
- multi-object reasoning

---

# 🧠 Design Philosophy

> Keep the perception module focused, stable, and easy to integrate.

Rather than solving every robotics task inside one node, the system provides:

```
Clean perception output
Stable runtime behavior
Reusable interfaces
```

---

# 🎯 Intended Use

This project is intended for:

- ABU Robocon 2026
- robotics perception systems
- ROS 2 development
- deployable computer vision pipelines

---

# 🧪 Maintainer

Project:

```
OpenVisionABU26
```

Domain:

```
Robotics Perception
Computer Vision
ROS 2 Systems
```

---

# 📌 Version

```
Version: v1.0
Status: Stable perception baseline
Deployment readiness: Verified
```
