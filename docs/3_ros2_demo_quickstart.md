# ABU YOLO ROS2 — Quick Demo Guide

This document provides a **short and practical guide** to run the perception demo for the ABU Robocon vision system.

The goal of this demo is to verify that the system can:

- Receive camera input
- Run YOLO detection
- Publish detection results
- Display annotated images

---

# 1. System Requirements

Environment:

```
Ubuntu 24.04
ROS2 Jazzy
OpenCV
ONNX Runtime
CUDA (optional but recommended)
```

Hardware:

```
Camera device (USB / Laptop camera)
GPU recommended
```

---

# 2. Workspace Setup

Open terminal and run:

```
cd ~/ros2_ws

source /opt/ros/jazzy/setup.zsh

colcon build

source install/setup.zsh
```

---

# 3. Run the Demo (Recommended Method)

Use the launch file to start the full system.

```
ros2 launch abu_yolo_ros yolo.launch.py
```

This command will start:

```
Camera node
YOLO detection node
Configuration loading
```

---

# 4. Verify Detection Output

Open a new terminal.

```
source ~/ros2_ws/install/setup.zsh
```

Check available topics:

```
ros2 topic list
```

Expected topics:

```
/image_raw
/yolo/image_annotated
/yolo/detections
```

---

# 5. Check Detection Messages

```
ros2 topic echo /yolo/detections
```

Expected output:

```
detections:
- bbox
- class_id
- confidence
```

If no object is detected:

```
detections: []
```

This is normal.

---

# 6. View Detection Visualization

Run:

```
ros2 run rqt_image_view rqt_image_view
```

Select topic:

```
/yolo/image_annotated
```

You should see:

```
Bounding boxes on detected objects
```

---

# 7. Check System Performance

Check detection frequency:

```
ros2 topic hz /yolo/detections
```

Expected:

```
10–30 Hz
```

---

# 8. Stop the Demo

Press:

```
Ctrl + C
```

---

# 9. Troubleshooting

## Camera not detected

```
Check device path:

ls /dev/video*
```

---

## No detection output

Possible causes:

```
Object not visible
Confidence threshold too high
Incorrect model path
```

---

## vision_msgs error

Install dependency:

```
sudo apt install ros-jazzy-vision-msgs
```

---

# 10. One-Line Demo Command

For quick testing:

```
ros2 launch abu_yolo_ros yolo.launch.py
```

---

# End of document

