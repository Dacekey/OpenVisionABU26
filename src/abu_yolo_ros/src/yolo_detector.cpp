#include "abu_yolo_ros/yolo_detector.hpp"
#include <opencv2/dnn.hpp>

#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <fstream>

static std::vector<int> applyNMS(
    const std::vector<cv::Rect>& boxes,
    const std::vector<float>& scores,
    float score_threshold,
    float nms_threshold
) {
    std::vector<int> indices;

    cv::dnn::NMSBoxes(
        boxes,
        scores,
        score_threshold,
        nms_threshold,
        indices
    );

    return indices;
}

YOLODetector::YOLODetector(const std::string& model_path,
                           const std::string& class_names_path,
                           bool use_gpu)
    : env_(ORT_LOGGING_LEVEL_WARNING, "YOLODetector") {
    session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

    if (use_gpu) {
        try {
            OrtCUDAProviderOptions cuda_options{};
            session_options_.AppendExecutionProvider_CUDA(cuda_options);
            std::cout << "[INFO] Using GPU (CUDA)" << std::endl;
        } catch (const std::exception& e) {
            std::cout << "[WARN] Failed to enable CUDA, fallback to CPU: " << e.what() << std::endl;
        }
    } else {
        std::cout << "[INFO] Using CPU" << std::endl;
    }

    session_ = std::make_unique<Ort::Session>(
        env_,
        model_path.c_str(),
        session_options_
    );

    Ort::AllocatorWithDefaultOptions allocator;

    const size_t num_inputs = session_->GetInputCount();
    const size_t num_outputs = session_->GetOutputCount();

    input_names_.reserve(num_inputs);
    input_shapes_.reserve(num_inputs);
    output_names_.reserve(num_outputs);
    output_shapes_.reserve(num_outputs);

    for (size_t i = 0; i < num_inputs; ++i) {
        auto name_ptr = session_->GetInputNameAllocated(i, allocator);
        input_names_.emplace_back(name_ptr.get());

        auto type_info = session_->GetInputTypeInfo(i);
        auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
        input_shapes_.push_back(tensor_info.GetShape());
    }

    for (size_t i = 0; i < num_outputs; ++i) {
        auto name_ptr = session_->GetOutputNameAllocated(i, allocator);
        output_names_.emplace_back(name_ptr.get());

        auto type_info = session_->GetOutputTypeInfo(i);
        auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
        output_shapes_.push_back(tensor_info.GetShape());
    }

    std::cout << "[INFO] Model loaded successfully: " << model_path << std::endl;

    // std::ifstream ifs("../models/fptu_abu26_detect.names");
    std::ifstream ifs(class_names_path);


    if (!ifs.is_open()) {
        std::cerr << "[WARN] Cannot open class names file" << std::endl;
    } else {

        std::string line;

        while (std::getline(ifs, line)) {
            class_names_.push_back(line);
        }

        std::cout
            << "[INFO] Loaded "
            << class_names_.size()
            << " class names"
            << std::endl;
    }
}

void YOLODetector::printModelInfo() const {
    std::cout << "========== MODEL INFO ==========" << std::endl;

    std::cout << "Inputs: " << input_names_.size() << std::endl;
    for (size_t i = 0; i < input_names_.size(); ++i) {
        std::cout << "  [" << i << "] " << input_names_[i] << " shape: [";
        for (size_t j = 0; j < input_shapes_[i].size(); ++j) {
            std::cout << input_shapes_[i][j];
            if (j + 1 < input_shapes_[i].size()) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }

    std::cout << "Outputs: " << output_names_.size() << std::endl;
    for (size_t i = 0; i < output_names_.size(); ++i) {
        std::cout << "  [" << i << "] " << output_names_[i] << " shape: [";
        for (size_t j = 0; j < output_shapes_[i].size(); ++j) {
            std::cout << output_shapes_[i][j];
            if (j + 1 < output_shapes_[i].size()) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }

    std::cout << "===============================" << std::endl;
}

std::vector<float> YOLODetector::preprocess(const cv::Mat& image) const {
    cv::Mat resized;
    cv::resize(image, resized, cv::Size(640, 640));

    cv::Mat rgb;
    cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);

    rgb.convertTo(rgb, CV_32F, 1.0 / 255.0);

    std::vector<float> input_tensor_values(1 * 3 * 640 * 640);
    const int channels = 3;
    const int height = 640;
    const int width = 640;

    for (int c = 0; c < channels; ++c) {
        for (int h = 0; h < height; ++h) {
            for (int w = 0; w < width; ++w) {
                input_tensor_values[c * height * width + h * width + w] =
                    rgb.at<cv::Vec3f>(h, w)[c];
            }
        }
    }

    return input_tensor_values;
}

std::vector<Detection> YOLODetector::infer(const cv::Mat& image) const {
    if (image.empty()) {
        throw std::runtime_error("Input image is empty.");
    }

    auto input_tensor_values = preprocess(image);
    std::vector<int64_t> input_shape = {1, 3, 640, 640};

    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(
        OrtArenaAllocator,
        OrtMemTypeDefault
    );

    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info,
        input_tensor_values.data(),
        input_tensor_values.size(),
        input_shape.data(),
        input_shape.size()
    );

    std::vector<const char*> input_names_cstr;
    for (const auto& name : input_names_) {
        input_names_cstr.push_back(name.c_str());
    }

    std::vector<const char*> output_names_cstr;
    for (const auto& name : output_names_) {
        output_names_cstr.push_back(name.c_str());
    }

    auto output_tensors = session_->Run(
        Ort::RunOptions{nullptr},
        input_names_cstr.data(),
        &input_tensor,
        1,
        output_names_cstr.data(),
        output_names_cstr.size()
    );

    float* output_data = output_tensors[0].GetTensorMutableData<float>();
    auto output_info = output_tensors[0].GetTensorTypeAndShapeInfo();
    auto output_shape = output_info.GetShape();

    const int num_predictions = output_shape[2];
    const int num_classes = output_shape[1] - 4;

    const float conf_threshold = 0.25f;
    const float score_threshold = 0.25f;
    const float nms_threshold = 0.45f;

    std::vector<Detection> raw_detections;
    std::vector<cv::Rect> boxes;
    std::vector<float> scores;

    const float scale_x = static_cast<float>(image.cols) / 640.0f;
    const float scale_y = static_cast<float>(image.rows) / 640.0f;

    for (int i = 0; i < num_predictions; ++i) {
        float x = output_data[i];
        float y = output_data[num_predictions + i];
        float w = output_data[2 * num_predictions + i];
        float h = output_data[3 * num_predictions + i];

        int best_class = -1;
        float best_score = 0.0f;

        for (int c = 0; c < num_classes; ++c) {
            float score = output_data[(4 + c) * num_predictions + i];
            if (score > best_score) {
                best_score = score;
                best_class = c;
            }
        }

        if (best_score <= conf_threshold) {
            continue;
        }

        Detection det;
        det.class_id = best_class;
        det.confidence = best_score;
        det.x = x * scale_x;
        det.y = y * scale_y;
        det.w = w * scale_x;
        det.h = h * scale_y;

        raw_detections.push_back(det);

        int x1 = static_cast<int>(det.x - det.w / 2.0f);
        int y1 = static_cast<int>(det.y - det.h / 2.0f);
        int width = static_cast<int>(det.w);
        int height = static_cast<int>(det.h);

        boxes.emplace_back(x1, y1, width, height);
        scores.push_back(det.confidence);
    }

    std::vector<int> keep_indices = applyNMS(boxes, scores, score_threshold, nms_threshold);

    std::vector<Detection> final_detections;
    final_detections.reserve(keep_indices.size());

    for (int idx : keep_indices) {
        final_detections.push_back(raw_detections[idx]);
    }

    return final_detections;
}

cv::Mat YOLODetector::drawDetections(
    const cv::Mat& image,
    const std::vector<Detection>& detections
) const {
    if (image.empty()) {
        throw std::runtime_error("Input image is empty.");
    }

    cv::Mat vis_image = image.clone();

    for (const auto& det : detections) {
        int x1 = static_cast<int>(det.x - det.w / 2.0f);
        int y1 = static_cast<int>(det.y - det.h / 2.0f);
        int x2 = static_cast<int>(det.x + det.w / 2.0f);
        int y2 = static_cast<int>(det.y + det.h / 2.0f);

        cv::rectangle(
            vis_image,
            cv::Point(x1, y1),
            cv::Point(x2, y2),
            cv::Scalar(0, 255, 0),
            2
        );

        std::string label;
        if (det.class_id >= 0 && det.class_id < static_cast<int>(class_names_.size())) {
            label = class_names_[det.class_id];
        } else {
            label = "cls:" + std::to_string(det.class_id);
        }

        label += " " + cv::format("%.2f", det.confidence);

        cv::putText(
            vis_image,
            label,
            cv::Point(x1, std::max(15, y1 - 5)),
            cv::FONT_HERSHEY_SIMPLEX,
            0.5,
            cv::Scalar(0, 255, 0),
            1
        );
    }

    return vis_image;
}

cv::Mat YOLODetector::inferAndDraw(const cv::Mat& image) const {
    auto detections = infer(image);
    return drawDetections(image, detections);
}