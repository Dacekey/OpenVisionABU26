#pragma once

namespace abu_yolo_ros {

struct Detection {
    int class_id;
    float confidence;
    float x;
    float y;
    float w;
    float h;
};

}  // namespace abu_yolo_ros
