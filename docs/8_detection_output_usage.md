# Detection Output Usage Guide

This document explains how to use the detection outputs from the YOLO
perception system for downstream robotics tasks such as navigation,
decision-making, and manipulation.

The goal is to help developers understand **what the detection node
publishes** and **how to integrate those outputs into other robot
modules**.

------------------------------------------------------------------------

# Overview

The perception system publishes structured detection results through ROS
2 topics.

These outputs are designed to be consumed by:

-   navigation modules
-   planning modules
-   robot control logic
-   task execution systems
-   monitoring or logging systems
-   future perception layers (tracking, grouping, filtering)

------------------------------------------------------------------------

# Main Detection Outputs

## Topic: /yolo/detections

Type:

``` text
vision_msgs/msg/Detection2DArray
```

This is the primary machine-readable output.

Each message contains:

``` text
bounding box
class id
confidence score
timestamp
frame reference
```

------------------------------------------------------------------------

## Topic: /yolo/image_annotated

Type:

``` text
sensor_msgs/msg/Image
```

Purpose:

``` text
visual debugging
system monitoring
demo visualization
```

This topic is typically **not used for robot logic**, only for
visualization.

------------------------------------------------------------------------

# Detection Message Structure (Simplified)

``` text
Detection2DArray
 ├── header
 └── detections
       ├── bbox
       │     ├── center_x
       │     ├── center_y
       │     ├── width
       │     └── height
       ├── class_id
       └── confidence
```

------------------------------------------------------------------------

# How to Subscribe to Detection Output

Python example:

``` python
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray

class DetectionSubscriber(Node):

    def __init__(self):
        super().__init__('detection_subscriber')

        self.subscription = self.create_subscription(
            Detection2DArray,
            '/yolo/detections',
            self.callback,
            10
        )

    def callback(self, msg):

        for detection in msg.detections:

            bbox = detection.bbox
            center_x = bbox.center.position.x
            center_y = bbox.center.position.y

            self.get_logger().info(
                f"Detected object at ({center_x}, {center_y})"
            )

def main():
    rclpy.init()
    node = DetectionSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

------------------------------------------------------------------------

# Typical Use Cases

## 1. Navigation

Goal:

``` text
move robot toward detected object
```

Logic example:

``` text
if object detected:
    compute target position
    send navigation command
```

------------------------------------------------------------------------

## 2. Object Selection

Goal:

``` text
choose which object to interact with
```

Logic example:

``` text
select detection with highest confidence
```

or

``` text
select closest detection
```

------------------------------------------------------------------------

## 3. Filtering Fake Objects

Goal:

``` text
ignore unwanted detections
```

Logic example:

``` text
if class == fake:
    ignore
else:
    process
```

This is especially relevant for competition tasks involving:

``` text
real vs fake objects
team color filtering
target validation
```

------------------------------------------------------------------------

## 4. Distance Estimation

Goal:

``` text
estimate how far the object is
```

Common method:

``` text
use bounding box size
camera calibration
depth sensor
```

Example:

``` text
larger bounding box → closer object
```

------------------------------------------------------------------------

## 5. Pick-and-Place Logic

Goal:

``` text
trigger robot arm to pick object
```

Logic example:

``` text
if object centered and distance < threshold:
    execute pick
```

------------------------------------------------------------------------

# Coordinate Usage

Bounding box center is often used as the main control reference.

``` text
center_x
center_y
```

Typical control logic:

``` text
if center_x < left_threshold:
    move robot left

if center_x > right_threshold:
    move robot right
```

------------------------------------------------------------------------

# Recommended Processing Steps

``` text
Receive detection
    ↓
Filter detections
    ↓
Select target
    ↓
Compute action
    ↓
Send command
```

------------------------------------------------------------------------

# Integration Example

``` text
Perception Node
      ↓
/yolo/detections
      ↓
Decision Node
      ↓
Navigation Node
      ↓
Motor Controller
```

------------------------------------------------------------------------

# Performance Considerations

Important factors:

``` text
detection latency
camera frame rate
CPU / GPU load
communication delay
```

Recommended:

``` text
process only necessary detections
avoid heavy computation inside callback
use asynchronous logic when possible
```

------------------------------------------------------------------------

# Debugging Tips

Check detection rate:

``` bash
ros2 topic hz /yolo/detections
```

Check detection content:

``` bash
ros2 topic echo /yolo/detections
```

------------------------------------------------------------------------

# Design Principle

Detection output should remain:

``` text
simple
structured
hardware-independent
reusable
```

This allows the same perception system to support multiple robot
behaviors without modification.

------------------------------------------------------------------------

# Summary

The detection output is designed to act as a **standard interface**
between perception and robot behavior.

Typical downstream workflow:

``` text
Detection
    ↓
Decision
    ↓
Action
```

This separation keeps the system modular, maintainable, and scalable.
