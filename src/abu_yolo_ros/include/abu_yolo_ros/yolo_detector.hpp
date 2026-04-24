#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>

#include "abu_yolo_ros/detection_types.hpp"

namespace abu_yolo_ros {

class YOLODetector {
public:
    YOLODetector(const std::string& model_path,
             const std::string& class_names_path,
             bool use_gpu = true);

    void printModelInfo() const;

    std::vector<Detection> infer(const cv::Mat& image) const;

    cv::Mat drawDetections(const cv::Mat& image,
                           const std::vector<Detection>& detections) const;

    std::string getClassLabel(int class_id) const;

    // giu tam de tuong thich code cu
    cv::Mat inferAndDraw(const cv::Mat& image) const;

private:
    std::vector<float> preprocess(const cv::Mat& image) const;

private:
    Ort::Env env_;
    Ort::SessionOptions session_options_;
    std::unique_ptr<Ort::Session> session_;

    std::vector<std::string> input_names_;
    std::vector<std::string> output_names_;
    std::vector<std::string> class_names_;
    std::vector<std::vector<int64_t>> input_shapes_;
    std::vector<std::vector<int64_t>> output_shapes_;
};

}  // namespace abu_yolo_ros
