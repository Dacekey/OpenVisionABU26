#pragma once

#include <vector>

#include "abu_yolo_ros/detection_types.hpp"

namespace abu_yolo_ros {

// Future module for assigning/filtering detections by team color cues.
class TeamColorFilter {
public:
    TeamColorFilter() = default;
    ~TeamColorFilter() = default;

    std::vector<Detection> filter(
        const std::vector<Detection>& detections) const
    {
        return detections;
    }
};

}  // namespace abu_yolo_ros
