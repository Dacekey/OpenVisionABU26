#pragma once

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "abu_yolo_ros/detection_types.hpp"

namespace abu_yolo_ros {

struct InferenceBackendConfig {
    float conf_threshold = 0.25F;
    float nms_threshold = 0.45F;
};

class InferenceBackend {
public:
    virtual ~InferenceBackend() = default;

    virtual bool load() = 0;
    virtual std::vector<Detection> infer(const cv::Mat& bgr_image) = 0;
    virtual std::string backendName() const = 0;
    virtual bool isLoaded() const = 0;
    virtual std::string lastError() const = 0;
};

}  // namespace abu_yolo_ros
