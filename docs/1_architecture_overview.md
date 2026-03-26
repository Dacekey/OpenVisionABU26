# Architecture Overview

This document describes the high-level architecture of the
OpenVisionABU26 perception system.

------------------------------------------------------------------------

## System Workflow

``` text
Model
  ↓
Inference Engine
  ↓
ROS2 Node
  ↓
Detection Topics
  ↓
Robot Modules
```

------------------------------------------------------------------------

## Components

### 1. YOLO Model

Responsible for detecting objects from images.

Format:

``` text
ONNX
```

------------------------------------------------------------------------

### 2. Inference Engine

A C++ engine that runs the model using:

-   ONNX Runtime
-   OpenCV
-   CUDA (optional)

Responsibilities:

-   Load model
-   Run inference
-   Return detections

------------------------------------------------------------------------

### 3. ROS2 Detection Node

Responsibilities:

-   Subscribe to camera topic
-   Run inference
-   Publish detection results

------------------------------------------------------------------------

## Data Flow

``` text
Camera
  ↓
/image_raw
  ↓
YOLO Detection Node
  ↓
/yolo/detections
  ↓
Planning / Control
```

------------------------------------------------------------------------

## Design Principles

-   Modular
-   Reproducible
-   Deployable
-   Hardware-independent
