#pragma once

#include <vector>

#include "abu_yolo_ros/detection_types.hpp"

namespace abu_yolo_ros {

// Future module for associating detections across frames to stabilize outputs.
class TemporalTracker {
public:
    TemporalTracker() = default;
    ~TemporalTracker() = default;

    std::vector<Detection> update(const std::vector<Detection>& detections)
    {
        return detections;
    }
};

}  // namespace abu_yolo_ros
