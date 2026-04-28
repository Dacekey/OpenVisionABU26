#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "abu_yolo_ros/inference_backend.hpp"
#include "abu_yolo_ros/onnx_runtime_backend.hpp"
#include "abu_yolo_ros/tensorrt_backend.hpp"
#include "abu_yolo_ros/detection_types.hpp"

namespace abu_yolo_ros {

struct YoloDetectorConfig {
    std::string requested_backend = "onnxruntime";
    std::string fallback_backend = "onnxruntime";
    bool allow_fallback = true;
    std::string class_names_path;
    OnnxRuntimeBackendConfig onnx;
    TensorRtBackendConfig tensorrt;
};

class YOLODetector {
public:
    explicit YOLODetector(YoloDetectorConfig config);

    void printModelInfo() const;

    std::vector<Detection> infer(const cv::Mat& image);

    cv::Mat drawDetections(const cv::Mat& image,
                           const std::vector<Detection>& detections) const;

    std::string getClassLabel(int class_id) const;

    // giu tam de tuong thich code cu
    cv::Mat inferAndDraw(const cv::Mat& image);

    std::string backendName() const;
    std::string requestedBackendName() const;
    bool usingFallbackBackend() const;

private:
    std::unique_ptr<InferenceBackend> makeBackend(
        const std::string& backend_name) const;
    static std::string normalizeBackendName(
        const std::string& backend_name);
    void loadClassNames();
    void loadBackendOrThrow();

private:
    YoloDetectorConfig config_;
    std::unique_ptr<InferenceBackend> backend_;
    std::string requested_backend_name_;
    std::string active_backend_name_;
    bool using_fallback_backend_ = false;
    std::vector<std::string> class_names_;
};

}  // namespace abu_yolo_ros
