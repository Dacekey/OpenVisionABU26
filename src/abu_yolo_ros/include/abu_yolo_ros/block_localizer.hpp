#pragma once

#include <vector>

#include "abu_yolo_ros/detection_types.hpp"

namespace abu_yolo_ros {

// Future module for estimating block position from 2D detections and camera geometry.
class BlockLocalizer {
public:
    struct LocalizedBlock {
        bool valid{false};
    };

    BlockLocalizer() = default;
    ~BlockLocalizer() = default;

    std::vector<LocalizedBlock> localize(
        const std::vector<Detection>&) const
    {
        return {};
    }
};

}  // namespace abu_yolo_ros
