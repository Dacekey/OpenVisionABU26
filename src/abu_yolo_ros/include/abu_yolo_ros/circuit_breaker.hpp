#pragma once

namespace abu_yolo_ros {

// Future module for guarding downstream perception decisions during invalid or unstable states.
class CircuitBreaker {
public:
    CircuitBreaker() = default;
    ~CircuitBreaker() = default;

    bool allow() const
    {
        return true;
    }
};

}  // namespace abu_yolo_ros
