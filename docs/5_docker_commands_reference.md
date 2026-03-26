# Docker Commands Guide --- OpenVisionABU26

This document provides a complete reference of Docker commands used in
the OpenVisionABU26 perception system.

System stack:

-   ROS2 Jazzy
-   YOLO (ONNX Runtime)
-   OpenCV
-   USB Camera
-   Docker deployment

------------------------------------------------------------------------

# 1. Build Docker Image

``` bash
docker build -t openvision:latest .
```

------------------------------------------------------------------------

# 2. Run Container (Production Mode)

``` bash
docker run -it --rm   --device=/dev/video0   --network host   openvision:latest bash
```

------------------------------------------------------------------------

# 3. Setup ROS2 Environment

``` bash
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
```

------------------------------------------------------------------------

# 4. Launch the Perception System

``` bash
ros2 launch abu_yolo_ros yolo.launch.py
```

------------------------------------------------------------------------

# 5. Monitor With Second Terminal

``` bash
docker ps
docker exec -it <container_id> bash
```

Then:

``` bash
ros2 node list
ros2 topic list
ros2 topic hz /image_raw
ros2 topic echo /yolo/detections
```

------------------------------------------------------------------------

# 6. Debugging Commands

Check containers:

``` bash
docker ps
```

Check logs:

``` bash
docker logs <container_id>
```

Check camera:

``` bash
ls /dev/video*
```

------------------------------------------------------------------------

# 7. Cleanup Commands

Remove unused images:

``` bash
docker image prune
```

Remove build cache:

``` bash
docker builder prune
```

Check disk usage:

``` bash
docker system df
```

------------------------------------------------------------------------

# 8. Save Image For Deployment

``` bash
docker save -o openvision.tar openvision:latest
```

Load image:

``` bash
docker load -i openvision.tar
```

------------------------------------------------------------------------

# Typical Workflow

1.  Build Docker
2.  Run container
3.  Source ROS
4.  Launch system
5.  Monitor with second terminal
6.  Debug if needed
7.  Cleanup images

------------------------------------------------------------------------

Status:

Docker validated\
ROS2 integration validated\
Camera pipeline validated\
YOLO detection validated
