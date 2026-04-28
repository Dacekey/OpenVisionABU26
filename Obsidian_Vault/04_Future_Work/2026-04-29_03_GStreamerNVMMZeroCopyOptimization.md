# GStreamer, nvv4l2camerasrc, NVMM, DMA, and Zero-Copy Optimization Plan

## 1. Overview

This future work item covers possible camera-path performance optimization for Jetson deployment.

Topics in scope:

- GStreamer camera pipeline
- Jetson-specific `nvv4l2camerasrc`
- NVMM memory buffers
- DMA / zero-copy transfer
- reducing camera-to-GPU latency

This is a performance optimization topic, not a core perception logic task.

The purpose is to evaluate whether camera ingestion and memory movement can be improved later, after the main inference backend path is validated on the actual Jetson target.

## 2. Current Decision

Current decision:

For now, OpenVision-v3 should not integrate GStreamer `nvv4l2camerasrc` plus NVMM or zero-copy into the main runtime.

Reason:

The current pipeline already reaches approximately realtime behavior in the ONNX Runtime baseline, around `29-30 FPS`, with total mean latency around `32-33 ms` in the observed benchmark runs.

At this stage, the current bottleneck has not yet been proven to be camera capture or memory transfer.

Because of that, adding Jetson-specific camera optimization now would be premature.

## 3. Why Not Add It Now

Reasons to postpone this work:

- it adds Jetson-specific complexity
- it may reduce portability between laptop development and Jetson runtime
- it introduces camera format handling issues such as `NV12`, `YUYV`, `BGR`, and `RGB` conversion
- it can make debugging harder than the current `usb_cam` and standard ROS image path
- true zero-copy is only valuable if the runtime can keep data on GPU or NVMM through preprocessing and TensorRT inference

Current OpenVision-v3 still includes several CPU-side stages:

- TeamColorFilter
- HSV mask generation
- contour processing
- KFSInstanceAggregator
- debug image drawing
- 3D Localizer
- Localization Stabilizer

If the frame must be copied back to CPU for these operations, the benefit of NVMM or zero-copy becomes smaller.

In other words, camera-side zero-copy is most valuable when the rest of the hot path is also designed to preserve GPU residency as long as possible.

## 4. When To Revisit This

This should be revisited only after the following conditions are met:

1. Jetson Nano environment is ready.
2. TensorRT `.engine` is built on the Jetson Nano.
3. OpenVision-v3 runs with `active=tensorrt`.
4. TensorRT benchmark is collected on the Jetson Nano.
5. Benchmark shows camera capture, preprocessing, or CPU/GPU memory copy is a significant bottleneck.

This decision should be based on the existing runtime benchmark system, not on assumption alone.

## 5. Conditions That Justify Implementing It

GStreamer, NVMM, DMA, and zero-copy work becomes worth doing when one or more of these conditions is true:

- TensorRT inference becomes fast enough that camera, preprocess, or copy overhead is a major part of total latency
- benchmark data shows capture, preprocess, or copy time consumes a meaningful percentage of total runtime
- the camera is CSI or MIPI and compatible with the Jetson accelerated camera path
- the runtime can preserve GPU or NVMM buffers long enough to actually reduce copies
- debug or CPU-side image processing can be minimized or separated from the high-speed inference path
- the team needs lower latency than the current standard ROS image path can provide

Without these conditions, the additional implementation complexity may not provide meaningful system benefit.

## 6. Possible Future Architecture

A possible future optimized path would look like:

Camera CSI / V4L2  
-> GStreamer `nvv4l2camerasrc`  
-> NVMM buffer  
-> CUDA / GPU preprocessing  
-> TensorRT inference  
-> minimal CPU copy  
-> KFS outputs

The ideal goal is camera-to-TensorRT with minimal CPU round-trip.

That architecture is only worthwhile if downstream runtime structure can preserve the benefit rather than forcing repeated GPU-to-CPU transfers.

## 7. Risks

Known risks of this optimization path:

- Jetson-specific dependency and plugin availability
- more difficult debugging
- format conversion mismatch
- zero-copy may not be true zero-copy if OpenCV CPU operations are still required
- integration may break portability with laptop development
- performance may not improve if inference remains the dominant bottleneck
- ROS image publishing and debug visualization may become more complicated

This is why the work should remain optional and benchmark-driven.

## 8. Success Criteria

If this optimization is implemented later, success should mean:

- the pipeline still publishes all existing topics
- KFS detection output remains consistent with the previous pipeline
- TensorRT remains `active=tensorrt`
- FPS improves or latency decreases in a meaningful way
- P95 and P99 latency improve
- detection stability does not regress
- debug path remains available, even if it is separated from the optimized inference path

The optimization should be judged by end-to-end runtime benefit, not by architectural complexity alone.

## 9. Future Work Checklist

- [ ] Complete TensorRT live benchmark on Jetson Nano
- [ ] Identify the actual bottleneck from the benchmark summary
- [ ] Measure camera capture, preprocess, and copy latency
- [ ] Confirm CSI camera and `nvv4l2camerasrc` compatibility
- [ ] Prototype GStreamer pipeline outside ROS first
- [ ] Prototype NVMM and CUDA preprocess path
- [ ] Compare standard ROS image path vs GStreamer and NVMM path
- [ ] Decide whether to integrate into OpenVision-v3 runtime

## 10. Current Status

Current status summary:

- not implemented now
- added to Future Work
- revisit only after Jetson TensorRT benchmark
- current priority remains TensorRT live validation and benchmark on Jetson Nano

This note does not claim that GStreamer, NVMM, DMA, or zero-copy optimization is already implemented.

It also does not claim that this optimization is necessary yet. The timing and value of this work depend on measured Jetson runtime bottlenecks after TensorRT is validated.
