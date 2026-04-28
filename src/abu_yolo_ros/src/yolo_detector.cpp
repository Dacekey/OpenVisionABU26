#include "abu_yolo_ros/yolo_detector.hpp"

#include <cctype>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace abu_yolo_ros {

YOLODetector::YOLODetector(YoloDetectorConfig config)
    : config_(std::move(config))
{
    loadClassNames();
    loadBackendOrThrow();
}

void YOLODetector::printModelInfo() const
{
    std::cout << "========== BACKEND INFO ==========" << std::endl;
    std::cout << "Requested backend: " << requested_backend_name_ << std::endl;
    std::cout << "Active backend: " << active_backend_name_ << std::endl;
    std::cout << "Using fallback: " << (using_fallback_backend_ ? "true" : "false") << std::endl;
    std::cout << "Class names: " << class_names_.size() << std::endl;
    std::cout << "==================================" << std::endl;
}

std::vector<Detection> YOLODetector::infer(const cv::Mat& image)
{
    if (!backend_ || !backend_->isLoaded()) {
        throw std::runtime_error("Inference backend is not loaded");
    }
    return backend_->infer(image);
}

cv::Mat YOLODetector::drawDetections(
    const cv::Mat& image,
    const std::vector<Detection>& detections) const
{
    if (image.empty()) {
        throw std::runtime_error("Input image is empty.");
    }

    cv::Mat vis_image = image.clone();

    for (const auto& det : detections) {
        const int x1 = static_cast<int>(det.x - det.w * 0.5F);
        const int y1 = static_cast<int>(det.y - det.h * 0.5F);
        const int x2 = static_cast<int>(det.x + det.w * 0.5F);
        const int y2 = static_cast<int>(det.y + det.h * 0.5F);

        cv::rectangle(
            vis_image,
            cv::Point(x1, y1),
            cv::Point(x2, y2),
            cv::Scalar(0, 255, 0),
            2);

        std::string label = getClassLabel(det.class_id);
        label += " " + cv::format("%.2f", det.confidence);

        cv::putText(
            vis_image,
            label,
            cv::Point(x1, std::max(15, y1 - 5)),
            cv::FONT_HERSHEY_SIMPLEX,
            0.5,
            cv::Scalar(0, 255, 0),
            1);
    }

    return vis_image;
}

std::string YOLODetector::getClassLabel(int class_id) const
{
    if (class_id >= 0 &&
        class_id < static_cast<int>(class_names_.size())) {
        return class_names_[class_id];
    }
    return "unknown";
}

cv::Mat YOLODetector::inferAndDraw(const cv::Mat& image)
{
    return drawDetections(image, infer(image));
}

std::string YOLODetector::backendName() const
{
    return active_backend_name_.empty() ? "unknown" : active_backend_name_;
}

std::string YOLODetector::requestedBackendName() const
{
    return requested_backend_name_;
}

bool YOLODetector::usingFallbackBackend() const
{
    return using_fallback_backend_;
}

std::unique_ptr<InferenceBackend> YOLODetector::makeBackend(
    const std::string& backend_name) const
{
    const std::string normalized = normalizeBackendName(backend_name);
    if (normalized == "onnxruntime") {
        return std::make_unique<OnnxRuntimeBackend>(config_.onnx);
    }
    if (normalized == "tensorrt") {
        return std::make_unique<TensorRtBackend>(config_.tensorrt);
    }

    std::ostringstream stream;
    stream << "Unsupported inference backend '" << backend_name << "'";
    throw std::runtime_error(stream.str());
}

std::string YOLODetector::normalizeBackendName(
    const std::string& backend_name)
{
    std::string normalized;
    normalized.reserve(backend_name.size());
    for (const char ch : backend_name) {
        normalized.push_back(static_cast<char>(std::tolower(
            static_cast<unsigned char>(ch))));
    }

    if (normalized == "onnx" || normalized == "onnxruntime") {
        return "onnxruntime";
    }
    if (normalized == "trt" || normalized == "tensorrt") {
        return "tensorrt";
    }
    return normalized;
}

void YOLODetector::loadClassNames()
{
    std::ifstream ifs(config_.class_names_path);
    if (!ifs.is_open()) {
        throw std::runtime_error(
            "Cannot open class names file: " + config_.class_names_path);
    }

    std::string line;
    while (std::getline(ifs, line)) {
        class_names_.push_back(line);
    }

    std::cout << "[INFO] Loaded "
              << class_names_.size()
              << " class names"
              << std::endl;
}

void YOLODetector::loadBackendOrThrow()
{
    requested_backend_name_ = normalizeBackendName(config_.requested_backend);
    const std::string fallback_backend_name =
        normalizeBackendName(config_.fallback_backend);

    auto try_load_backend = [&](const std::string& name) {
        auto backend = makeBackend(name);
        if (!backend->load()) {
            std::ostringstream stream;
            stream << "Failed to load backend '" << name
                   << "': " << backend->lastError();
            throw std::runtime_error(stream.str());
        }
        return backend;
    };

    try {
        backend_ = try_load_backend(requested_backend_name_);
        active_backend_name_ = backend_->backendName();
        using_fallback_backend_ = false;
        return;
    } catch (const std::exception& e) {
        if (!config_.allow_fallback ||
            fallback_backend_name.empty() ||
            fallback_backend_name == requested_backend_name_) {
            throw;
        }

        std::cerr << "[WARN] " << e.what() << std::endl;
        std::cerr << "[WARN] Backend '" << requested_backend_name_
                  << "' unavailable; falling back to '"
                  << fallback_backend_name
                  << "'"
                  << std::endl;
    }

    backend_ = try_load_backend(fallback_backend_name);
    active_backend_name_ = backend_->backendName();
    using_fallback_backend_ = true;
}

}  // namespace abu_yolo_ros
