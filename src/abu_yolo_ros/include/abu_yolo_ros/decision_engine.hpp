#pragma once

#include <vector>

#include "abu_yolo_ros/detection_types.hpp"

namespace abu_yolo_ros {

// Future module for turning filtered perception outputs into task-level decisions.
class DecisionEngine {
public:
    struct Decision {
        bool valid{false};
    };

    DecisionEngine() = default;
    ~DecisionEngine() = default;

    Decision evaluate(const std::vector<Detection>&) const
    {
        return {};
    }
};

}  // namespace abu_yolo_ros
