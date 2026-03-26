# System Requirements

This document lists the minimum and recommended hardware and software
requirements.

------------------------------------------------------------------------

## Operating System

Supported:

-   Ubuntu 22.04
-   Ubuntu 24.04

------------------------------------------------------------------------

## Minimum Requirements

Hardware:

-   CPU: 4 cores
-   RAM: 8 GB
-   Storage: 20 GB free
-   Camera: USB camera

Software:

-   Docker
-   Git

Note: How to install and setup Docker for beginner at **docker_setup.md**

------------------------------------------------------------------------

## Recommended Requirements

Hardware:

-   CPU: 6--8 cores
-   RAM: 16 GB
-   Storage: SSD
-   NVIDIA GPU

Software:

-   Docker
-   NVIDIA driver
-   NVIDIA Container Toolkit

Note: Setup GPU in Docker at **docker_gpu_setup.md**

------------------------------------------------------------------------

## Quick Verification

``` bash
docker --version
nvidia-smi
ls /dev/video0
```
