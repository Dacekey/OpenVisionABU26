#pragma once

#include <memory>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>

#include "abu_yolo_ros/inference_backend.hpp"

namespace abu_yolo_ros {

struct OnnxRuntimeBackendConfig : public InferenceBackendConfig {
    std::string model_path;
    bool use_gpu = true;
    int input_width = 640;
    int input_height = 640;
};

class OnnxRuntimeBackend : public InferenceBackend {
public:
    explicit OnnxRuntimeBackend(OnnxRuntimeBackendConfig config);

    bool load() override;
    std::vector<Detection> infer(const cv::Mat& bgr_image) override;
    std::string backendName() const override;
    bool isLoaded() const override;
    std::string lastError() const override;

private:
    OnnxRuntimeBackendConfig config_;
    bool loaded_ = false;
    std::string last_error_;
    Ort::Env env_;
    Ort::SessionOptions session_options_;
    std::unique_ptr<Ort::Session> session_;
    std::vector<std::string> input_names_;
    std::vector<std::string> output_names_;
};

}  // namespace abu_yolo_ros
