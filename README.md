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

## Developer Utilities

The repository also builds two standalone image debug utility nodes for camera
and topic validation. These are developer tools only and are not launched by
default in the main perception pipeline.

- `image_passthrough_node`
  republishes `/image_raw` to `/yolo/image_debug` unchanged
- `image_bgr_debug_node`
  converts the raw image feed to `/yolo/image_bgr_debug` for BGR inspection

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

The OpenVision-v3 perception runtime now also uses sensor-style QoS on its realtime vision streams and adds local runtime hardening inside `yolo_detection_node`: an inference mutex with optional busy-frame drop, plus a lightweight circuit breaker for repeated inference failures/timeouts. When that circuit breaker is OPEN, the node skips inference and skips publishing fresh perception outputs for that frame rather than publishing empty arrays. No separate health/status topic is emitted yet.

## Inference Backend Selection

`yolo_detection_node` now supports a config-driven inference backend selection layer.

- Current default backend: ONNX Runtime
- Optional backend: TensorRT Runtime
- ONNX Runtime remains the fallback backend

Backend selection lives under `inference.*` in `src/abu_yolo_ros/config/yolo_detection.yaml`:

```yaml
inference.backend: "onnxruntime"
inference.fallback_backend: "onnxruntime"
inference.allow_fallback: true
inference.onnx.model_path: "/home/dacekey/openvision_ros2_ws_v2/src/abu_yolo_ros/models/fptu_abu26_detect_yolo_v1.onnx"
inference.onnx.use_gpu: true
inference.tensorrt.engine_path: "/home/dacekey/openvision_ros2_ws_v2/src/abu_yolo_ros/models/fptu_abu26_detect_yolo_v1.engine"
inference.tensorrt.input_name: ""
inference.tensorrt.output_name: ""
inference.tensorrt.input_width: 640
inference.tensorrt.input_height: 640
inference.tensorrt.fp16: true
inference.tensorrt.workspace_size_mb: 1024
inference.tensorrt.allow_engine_rebuild: false
inference.tensorrt.onnx_model_path_for_build: "/home/dacekey/openvision_ros2_ws_v2/src/abu_yolo_ros/models/fptu_abu26_detect_yolo_v1.onnx"
inference.nms_threshold: 0.45
```

Backend switching is parameter-driven and does not change ROS message contracts, topic names, KFS aggregation, TeamColorFilter, DecisionEngine, 3D localization, or localization stabilization.

To run the default ONNX Runtime backend:

```bash
ros2 launch abu_yolo_ros yolo.launch.py
```

To request TensorRT Runtime:

```bash
ros2 launch abu_yolo_ros yolo.launch.py inference.backend:=tensorrt
```

If TensorRT support was not compiled, the engine file is missing, or TensorRT engine loading fails, the runtime can fall back to ONNX Runtime when `inference.allow_fallback=true`.

### Building a TensorRT Engine

The runtime primarily supports loading an existing TensorRT engine. In-runtime engine rebuild is not implemented, so the recommended path is to build the engine externally with `trtexec`.

Example:

```bash
trtexec \
  --onnx=src/abu_yolo_ros/models/fptu_abu26_detect_yolo_v1.onnx \
  --saveEngine=src/abu_yolo_ros/models/fptu_abu26_detect_yolo_v1.engine \
  --fp16 \
  --workspace=1024
```

TensorRT / `trtexec` flags vary by release, so verify the exact syntax with:

```bash
trtexec --help
```

The TensorRT engine must match:

- the model
- the input resolution
- the TensorRT version
- the target GPU architecture / Jetson target

### Runtime Benchmark

The same `yolo_detection_node` includes runtime benchmark instrumentation for direct ONNX Runtime vs TensorRT Runtime comparison. This is measurement-only instrumentation around the existing C++ runtime and does not change inference outputs, TeamColorFilter logic, DecisionEngine logic, KFS aggregation, 3D localization, localization stabilization, message contracts, or topic names.

Benchmark configuration lives under `runtime_benchmark.*` in `src/abu_yolo_ros/config/yolo_detection.yaml`:

```yaml
runtime_benchmark.enabled: true
runtime_benchmark.summary_interval_frames: 100
runtime_benchmark.warmup_frames: 20
runtime_benchmark.log_per_frame: false
runtime_benchmark.reset_after_summary: true
runtime_benchmark.include_publish_time: true
```

The current baseline benchmark measures these node-local stages inside `yolo_detection_node`:

- `image_callback_total_ms`
- `preprocess_ms`
- `inference_ms`
- `postprocess_ms`
- `team_color_filter_ms`
- `kfs_instance_aggregation_ms`
- `publish_ms`
- `total_ms`

For each measured stage, the benchmark computes:

- `mean_ms`
- `min_ms`
- `max_ms`
- `p50_ms`
- `p95_ms`
- `p99_ms`
- `sample_count`

It also reports:

- estimated FPS from `total_ms`
- warmup-skipped frame count
- busy-drop count from runtime safety
- inference failure count
- inference timeout count
- current circuit breaker state

The benchmark is intentionally lightweight:

- it uses `std::chrono` timing only
- it does not add new ROS topics
- it does not alter inference behavior or model outputs
- it works with both ONNX Runtime and TensorRT Runtime
- it does not implement NVMM or zero-copy by itself

Expected summary logs look like:

```text
Runtime benchmark backend=onnxruntime frames=100 fps=29.4 total mean=31.8ms p95=42.1ms p99=55.0ms | preprocess mean=2.1ms | infer mean=18.4ms p95=25.6ms | postprocess mean=1.8ms | color mean=1.2ms | kfs_agg mean=2.4ms | publish mean=0.6ms | dropped=0 timeout=0 failures=0 warmup_skipped=20 circuit=CLOSED
```

For backend comparison, keep the same:

- model
- image resolution
- input source
- hardware
- runtime config
- test conditions
- benchmark stages

NVMM / zero-copy optimization should only be considered later if the benchmark shows that camera conversion or copy overhead is a real bottleneck.

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
