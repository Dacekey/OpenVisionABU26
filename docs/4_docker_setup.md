# Docker Setup Guide

This document describes how to install and prepare Docker for running
the OpenVisionABU26 system.

------------------------------------------------------------------------

## Step 1 --- Install Docker

``` bash
sudo apt update
sudo apt install -y docker.io
```

Verify installation:

``` bash
docker --version
```

------------------------------------------------------------------------

## Step 2 --- Enable Docker Without sudo (Optional)

``` bash
sudo usermod -aG docker $USER
newgrp docker
```

------------------------------------------------------------------------

## Step 3 --- Verify Docker

``` bash
docker run hello-world
```

Expected:

``` text
Hello from Docker!
```

------------------------------------------------------------------------

## Step 4 --- Build Project Image

``` bash
docker build -t openvision:latest .
```

------------------------------------------------------------------------

## Step 5 --- Run Container

``` bash
docker run -it --rm \
  --device=/dev/video0 \
  --network host \
  openvision:latest bash
```

------------------------------------------------------------------------

## Optional --- GPU Mode

``` bash
docker run -it --rm \
  --gpus all \
  --device=/dev/video0 \
  --network host \
  openvision:latest bash
```
Note: If meeting error, check **docker_gpu_setup.md**