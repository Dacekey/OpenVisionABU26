#include "abu_yolo_ros/onnx_runtime_backend.hpp"

#include <iostream>
#include <stdexcept>

#include "abu_yolo_ros/yolo_inference_utils.hpp"

namespace abu_yolo_ros {

OnnxRuntimeBackend::OnnxRuntimeBackend(OnnxRuntimeBackendConfig config)
    : config_(std::move(config)),
      env_(ORT_LOGGING_LEVEL_WARNING, "OnnxRuntimeBackend")
{
}

bool OnnxRuntimeBackend::load()
{
    try {
        session_options_.SetGraphOptimizationLevel(
            GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

        if (config_.use_gpu) {
            try {
                OrtCUDAProviderOptions cuda_options{};
                session_options_.AppendExecutionProvider_CUDA(cuda_options);
                std::cout << "[INFO] ONNX Runtime backend using GPU (CUDA)" << std::endl;
            } catch (const std::exception& e) {
                std::cout << "[WARN] Failed to enable CUDA for ONNX Runtime, fallback to CPU: "
                          << e.what()
                          << std::endl;
            }
        } else {
            std::cout << "[INFO] ONNX Runtime backend using CPU" << std::endl;
        }

        session_ = std::make_unique<Ort::Session>(
            env_,
            config_.model_path.c_str(),
            session_options_);

        Ort::AllocatorWithDefaultOptions allocator;
        input_names_.clear();
        output_names_.clear();

        const size_t num_inputs = session_->GetInputCount();
        const size_t num_outputs = session_->GetOutputCount();
        input_names_.reserve(num_inputs);
        output_names_.reserve(num_outputs);

        for (size_t i = 0; i < num_inputs; ++i) {
            auto name_ptr = session_->GetInputNameAllocated(i, allocator);
            input_names_.emplace_back(name_ptr.get());
        }
        for (size_t i = 0; i < num_outputs; ++i) {
            auto name_ptr = session_->GetOutputNameAllocated(i, allocator);
            output_names_.emplace_back(name_ptr.get());
        }

        loaded_ = true;
        last_error_.clear();
        std::cout << "[INFO] ONNX Runtime backend loaded model: "
                  << config_.model_path
                  << std::endl;
        return true;
    } catch (const std::exception& e) {
        loaded_ = false;
        last_error_ = e.what();
        return false;
    }
}

std::vector<Detection> OnnxRuntimeBackend::infer(const cv::Mat& bgr_image)
{
    if (!loaded_ || !session_) {
        throw std::runtime_error("ONNX Runtime backend is not loaded");
    }
    if (bgr_image.empty()) {
        throw std::runtime_error("Input image is empty.");
    }

    std::vector<float> input_tensor_values = preprocessImageToChw(
        bgr_image,
        config_.input_width,
        config_.input_height);
    const std::vector<int64_t> input_shape = {
        1,
        3,
        config_.input_height,
        config_.input_width
    };

    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(
        OrtArenaAllocator,
        OrtMemTypeDefault);
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info,
        input_tensor_values.data(),
        input_tensor_values.size(),
        input_shape.data(),
        input_shape.size());

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
        output_names_cstr.size());

    if (output_tensors.empty()) {
        throw std::runtime_error("ONNX Runtime backend produced no outputs");
    }

    float* output_data = output_tensors[0].GetTensorMutableData<float>();
    const auto output_shape =
        output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();
    return decodeYoloDetections(
        output_data,
        output_shape,
        bgr_image.size(),
        config_,
        config_.input_width,
        config_.input_height);
}

std::string OnnxRuntimeBackend::backendName() const
{
    return "onnxruntime";
}

bool OnnxRuntimeBackend::isLoaded() const
{
    return loaded_;
}

std::string OnnxRuntimeBackend::lastError() const
{
    return last_error_;
}

}  // namespace abu_yolo_ros
