#pragma once

#include <cstdint>
#include <vector>

#include <opencv2/opencv.hpp>

#include "abu_yolo_ros/detection_types.hpp"
#include "abu_yolo_ros/inference_backend.hpp"

namespace abu_yolo_ros {

std::vector<float> preprocessImageToChw(
    const cv::Mat& image,
    int input_width,
    int input_height);

std::vector<Detection> decodeYoloDetections(
    const float* output_data,
    const std::vector<int64_t>& output_shape,
    const cv::Size& original_image_size,
    const InferenceBackendConfig& config,
    int input_width,
    int input_height);

}  // namespace abu_yolo_ros
