#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "abu_yolo_ros/inference_backend.hpp"

namespace abu_yolo_ros {

struct TensorRtBackendConfig : public InferenceBackendConfig {
    std::string engine_path;
    std::string input_name;
    std::string output_name;
    int input_width = 640;
    int input_height = 640;
    bool fp16 = true;
    int workspace_size_mb = 1024;
    bool allow_engine_rebuild = false;
    std::string onnx_model_path_for_build;
};

class TensorRtBackend : public InferenceBackend {
public:
    explicit TensorRtBackend(TensorRtBackendConfig config);
    ~TensorRtBackend() override;

    bool load() override;
    std::vector<Detection> infer(const cv::Mat& bgr_image) override;
    std::string backendName() const override;
    bool isLoaded() const override;
    std::string lastError() const override;

private:
    TensorRtBackendConfig config_;
    bool loaded_ = false;
    std::string last_error_;

#ifdef ABU_YOLO_ROS_HAS_TENSORRT
    struct Impl;
    std::unique_ptr<Impl> impl_;
#endif
};

}  // namespace abu_yolo_ros
