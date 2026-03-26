# GPU Setup Guide --- OpenVisionABU26 Docker

This document describes the exact procedure to enable GPU support for
the OpenVisionABU26 Docker system.

The steps below replicate the real setup workflow that was successfully
used during development.

------------------------------------------------------------------------

# Overview

Goal:

-   Enable Docker to access NVIDIA GPU
-   Run YOLO inference using GPU
-   Use ROS2 perception pipeline with hardware acceleration

Environment:

-   Ubuntu 24.04
-   NVIDIA GPU
-   Docker
-   ROS2 Jazzy
-   ONNX Runtime

------------------------------------------------------------------------

# Step 0 --- Verify GPU Driver on Host

Run:

``` bash
nvidia-smi
```

Expected:

``` text
GPU information table appears
Driver version is shown
```

If this command fails:

-   Install NVIDIA driver first
-   Reboot system

------------------------------------------------------------------------

# Step 1 --- Add NVIDIA Container Toolkit Repository

``` bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
```

``` bash
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```

------------------------------------------------------------------------

# Step 2 --- Update Package List

``` bash
sudo apt update
```

------------------------------------------------------------------------

# Step 3 --- Install NVIDIA Container Toolkit

``` bash
sudo apt install -y nvidia-container-toolkit
```

Verify:

``` bash
nvidia-ctk --version
```

------------------------------------------------------------------------

# Step 4 --- Configure Docker Runtime

``` bash
sudo nvidia-ctk runtime configure --runtime=docker
```

Expected output:

``` text
Wrote updated config to /etc/docker/daemon.json
```

------------------------------------------------------------------------

# Step 5 --- Restart Docker

``` bash
sudo systemctl restart docker
```

------------------------------------------------------------------------

# Step 6 --- Test GPU Inside Docker

Run:

``` bash
docker run --rm --gpus all nvidia/cuda:12.3.0-base-ubuntu22.04 nvidia-smi
```

Expected:

``` text
GPU information table appears inside container
```

If error occurs:

``` text
could not select device driver
```

Then:

-   Docker runtime not configured
-   Restart Docker again

------------------------------------------------------------------------

# Step 7 --- Run OpenVision Container with GPU

``` bash
docker run -it --rm   --gpus all   --device=/dev/video0   --network host   openvision:latest bash
```

------------------------------------------------------------------------

# Step 8 --- Setup ROS2 Environment

Inside container:

``` bash
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
```

------------------------------------------------------------------------

# Step 9 --- Launch Perception System

``` bash
ros2 launch abu_yolo_ros yolo.launch.py
```

Expected logs:

``` text
Using GPU: true
YOLO Detection Node started
```

------------------------------------------------------------------------

# Optional --- Verify GPU Usage

Run:

``` bash
nvidia-smi
```

You should see:

``` text
GPU memory usage increasing
Process name appears
```

------------------------------------------------------------------------

# Troubleshooting

## GPU works on host but not in Docker

Fix:

``` bash
sudo systemctl restart docker
```

------------------------------------------------------------------------

## Command not found: nvidia-ctk

Cause:

Repository not added

Solution:

Repeat Step 1

------------------------------------------------------------------------

## CUDA fallback to CPU

Example log:

``` text
Failed to enable CUDA
```

Cause:

Docker image missing CUDA runtime

Solution:

Use CUDA base image

------------------------------------------------------------------------

# Standard Production Command

``` bash
docker run -it --rm   --gpus all   --device=/dev/video0   --network host   openvision:latest bash
```

------------------------------------------------------------------------

# System Status After Setup

``` text
GPU driver installed
Docker GPU runtime configured
Container GPU access verified
YOLO GPU inference enabled
```

------------------------------------------------------------------------

End of document.
