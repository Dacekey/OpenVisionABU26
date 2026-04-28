#include "abu_yolo_ros/tensorrt_backend.hpp"

#include <cstdint>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "abu_yolo_ros/yolo_inference_utils.hpp"

#ifdef ABU_YOLO_ROS_HAS_TENSORRT
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#endif

namespace abu_yolo_ros {

#ifdef ABU_YOLO_ROS_HAS_TENSORRT
namespace {

class TrtLogger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override
    {
        if (severity > Severity::kWARNING) {
            return;
        }
        std::cerr << "[TensorRT] " << msg << std::endl;
    }
};

template <typename T>
struct InferDeleter {
    void operator()(T* ptr) const
    {
        if (ptr != nullptr) {
            ptr->destroy();
        }
    }
};

std::size_t elementSizeForDataType(const nvinfer1::DataType data_type)
{
    switch (data_type) {
    case nvinfer1::DataType::kFLOAT:
        return sizeof(float);
    case nvinfer1::DataType::kHALF:
        return sizeof(uint16_t);
    case nvinfer1::DataType::kINT32:
        return sizeof(int32_t);
    case nvinfer1::DataType::kINT8:
        return sizeof(int8_t);
#if NV_TENSORRT_MAJOR >= 8
    case nvinfer1::DataType::kBOOL:
        return sizeof(bool);
#endif
    default:
        throw std::runtime_error("Unsupported TensorRT data type");
    }
}

std::vector<int64_t> dimsToVector(const nvinfer1::Dims& dims)
{
    std::vector<int64_t> result;
    result.reserve(static_cast<std::size_t>(dims.nbDims));
    for (int i = 0; i < dims.nbDims; ++i) {
        result.push_back(static_cast<int64_t>(dims.d[i]));
    }
    return result;
}

std::size_t volumeOfDims(const nvinfer1::Dims& dims)
{
    std::size_t volume = 1;
    for (int i = 0; i < dims.nbDims; ++i) {
        if (dims.d[i] <= 0) {
            throw std::runtime_error("TensorRT tensor shape contains dynamic or invalid dimensions");
        }
        volume *= static_cast<std::size_t>(dims.d[i]);
    }
    return volume;
}

}  // namespace

struct TensorRtBackend::Impl {
    TrtLogger logger;
    std::unique_ptr<nvinfer1::IRuntime, InferDeleter<nvinfer1::IRuntime>> runtime;
    std::unique_ptr<nvinfer1::ICudaEngine, InferDeleter<nvinfer1::ICudaEngine>> engine;
    std::unique_ptr<nvinfer1::IExecutionContext, InferDeleter<nvinfer1::IExecutionContext>> context;
    cudaStream_t stream = nullptr;
    int input_binding_index = -1;
    int output_binding_index = -1;
    std::vector<void*> device_buffers;
    std::vector<std::size_t> buffer_sizes;
    std::vector<int64_t> output_shape;
};
#endif

TensorRtBackend::TensorRtBackend(TensorRtBackendConfig config)
    : config_(std::move(config))
{
#ifdef ABU_YOLO_ROS_HAS_TENSORRT
    impl_ = std::make_unique<Impl>();
#endif
}

TensorRtBackend::~TensorRtBackend()
{
#ifdef ABU_YOLO_ROS_HAS_TENSORRT
    if (impl_) {
        for (void* ptr : impl_->device_buffers) {
            if (ptr != nullptr) {
                cudaFree(ptr);
            }
        }
        if (impl_->stream != nullptr) {
            cudaStreamDestroy(impl_->stream);
        }
    }
#endif
}

bool TensorRtBackend::load()
{
#ifndef ABU_YOLO_ROS_HAS_TENSORRT
    loaded_ = false;
    last_error_ =
        "TensorRT support was not compiled. Rebuild with ENABLE_TENSORRT=ON and installed TensorRT libraries.";
    return false;
#else
    try {
        if (config_.allow_engine_rebuild) {
            std::cerr
                << "[WARN] TensorRT engine rebuild is not implemented in-runtime; "
                << "build the engine externally and set inference.tensorrt.engine_path"
                << std::endl;
        }

        std::ifstream engine_file(config_.engine_path, std::ios::binary);
        if (!engine_file.is_open()) {
            throw std::runtime_error(
                "Cannot open TensorRT engine file: " + config_.engine_path);
        }

        engine_file.seekg(0, std::ios::end);
        const std::streamsize engine_size = engine_file.tellg();
        engine_file.seekg(0, std::ios::beg);
        if (engine_size <= 0) {
            throw std::runtime_error("TensorRT engine file is empty");
        }

        std::vector<char> engine_data(static_cast<std::size_t>(engine_size));
        if (!engine_file.read(engine_data.data(), engine_size)) {
            throw std::runtime_error("Failed to read TensorRT engine file");
        }

        impl_->runtime.reset(nvinfer1::createInferRuntime(impl_->logger));
        if (!impl_->runtime) {
            throw std::runtime_error("Failed to create TensorRT runtime");
        }

        impl_->engine.reset(impl_->runtime->deserializeCudaEngine(
            engine_data.data(),
            engine_data.size()));
        if (!impl_->engine) {
            throw std::runtime_error("Failed to deserialize TensorRT engine");
        }

        impl_->context.reset(impl_->engine->createExecutionContext());
        if (!impl_->context) {
            throw std::runtime_error("Failed to create TensorRT execution context");
        }

        const int nb_bindings = impl_->engine->getNbBindings();
        if (nb_bindings < 2) {
            throw std::runtime_error("TensorRT engine must expose at least one input and one output");
        }

        impl_->device_buffers.assign(static_cast<std::size_t>(nb_bindings), nullptr);
        impl_->buffer_sizes.assign(static_cast<std::size_t>(nb_bindings), 0U);

        for (int i = 0; i < nb_bindings; ++i) {
            const bool is_input = impl_->engine->bindingIsInput(i);
            const std::string binding_name = impl_->engine->getBindingName(i);
            if (is_input) {
                if (!config_.input_name.empty() && binding_name != config_.input_name) {
                    continue;
                }
                if (impl_->input_binding_index < 0) {
                    impl_->input_binding_index = i;
                }
            } else {
                if (!config_.output_name.empty() && binding_name != config_.output_name) {
                    continue;
                }
                if (impl_->output_binding_index < 0) {
                    impl_->output_binding_index = i;
                }
            }
        }

        if (impl_->input_binding_index < 0 || impl_->output_binding_index < 0) {
            throw std::runtime_error(
                "Failed to resolve TensorRT input/output bindings from engine");
        }

        nvinfer1::Dims input_dims =
            impl_->engine->getBindingDimensions(impl_->input_binding_index);
        if (input_dims.nbDims != 4) {
            throw std::runtime_error("TensorRT input tensor must be 4D NCHW");
        }

        if (input_dims.d[0] == -1) {
            input_dims.d[0] = 1;
        }
        if (input_dims.d[2] == -1) {
            input_dims.d[2] = config_.input_height;
        }
        if (input_dims.d[3] == -1) {
            input_dims.d[3] = config_.input_width;
        }

        if (!impl_->context->setBindingDimensions(
                impl_->input_binding_index,
                input_dims)) {
            throw std::runtime_error("Failed to set TensorRT input binding dimensions");
        }

        const nvinfer1::Dims resolved_input_dims =
            impl_->context->getBindingDimensions(impl_->input_binding_index);
        const nvinfer1::Dims resolved_output_dims =
            impl_->context->getBindingDimensions(impl_->output_binding_index);
        impl_->output_shape = dimsToVector(resolved_output_dims);

        for (int i = 0; i < nb_bindings; ++i) {
            const nvinfer1::Dims dims = impl_->context->getBindingDimensions(i);
            const std::size_t bytes =
                volumeOfDims(dims) * elementSizeForDataType(impl_->engine->getBindingDataType(i));
            impl_->buffer_sizes[static_cast<std::size_t>(i)] = bytes;
            if (cudaMalloc(&impl_->device_buffers[static_cast<std::size_t>(i)], bytes) != cudaSuccess) {
                throw std::runtime_error("Failed to allocate TensorRT device buffer");
            }
        }

        if (cudaStreamCreate(&impl_->stream) != cudaSuccess) {
            throw std::runtime_error("Failed to create CUDA stream for TensorRT backend");
        }

        loaded_ = true;
        last_error_.clear();
        std::cerr << "[INFO] TensorRT backend loaded engine: "
                  << config_.engine_path
                  << std::endl;
        std::cerr << "[INFO] TensorRT backend input size: "
                  << config_.input_width
                  << "x"
                  << config_.input_height
                  << std::endl;
        if (resolved_input_dims.d[2] != config_.input_height ||
            resolved_input_dims.d[3] != config_.input_width) {
            std::cerr << "[WARN] TensorRT engine input shape differs from configured size; "
                      << "using resolved engine dimensions "
                      << resolved_input_dims.d[3] << "x" << resolved_input_dims.d[2]
                      << std::endl;
        }
        return true;
    } catch (const std::exception& e) {
        loaded_ = false;
        last_error_ = e.what();
        return false;
    }
#endif
}

std::vector<Detection> TensorRtBackend::infer(const cv::Mat& bgr_image)
{
#ifndef ABU_YOLO_ROS_HAS_TENSORRT
    (void)bgr_image;
    throw std::runtime_error(last_error_.empty() ? "TensorRT support not compiled" : last_error_);
#else
    if (!loaded_ || !impl_) {
        throw std::runtime_error("TensorRT backend is not loaded");
    }
    if (bgr_image.empty()) {
        throw std::runtime_error("Input image is empty.");
    }

    std::vector<float> input = preprocessImageToChw(
        bgr_image,
        config_.input_width,
        config_.input_height);

    const std::size_t input_bytes = input.size() * sizeof(float);
    if (input_bytes > impl_->buffer_sizes[static_cast<std::size_t>(impl_->input_binding_index)]) {
        throw std::runtime_error("TensorRT input buffer is smaller than expected");
    }

    if (cudaMemcpyAsync(
            impl_->device_buffers[static_cast<std::size_t>(impl_->input_binding_index)],
            input.data(),
            input_bytes,
            cudaMemcpyHostToDevice,
            impl_->stream) != cudaSuccess) {
        throw std::runtime_error("TensorRT host-to-device copy failed");
    }

    if (!impl_->context->enqueueV2(
            impl_->device_buffers.data(),
            impl_->stream,
            nullptr)) {
        throw std::runtime_error("TensorRT enqueueV2 failed");
    }

    const std::size_t output_bytes =
        impl_->buffer_sizes[static_cast<std::size_t>(impl_->output_binding_index)];
    std::vector<float> output(output_bytes / sizeof(float));
    if (cudaMemcpyAsync(
            output.data(),
            impl_->device_buffers[static_cast<std::size_t>(impl_->output_binding_index)],
            output_bytes,
            cudaMemcpyDeviceToHost,
            impl_->stream) != cudaSuccess) {
        throw std::runtime_error("TensorRT device-to-host copy failed");
    }
    if (cudaStreamSynchronize(impl_->stream) != cudaSuccess) {
        throw std::runtime_error("TensorRT CUDA stream synchronize failed");
    }

    return decodeYoloDetections(
        output.data(),
        impl_->output_shape,
        bgr_image.size(),
        config_,
        config_.input_width,
        config_.input_height);
#endif
}

std::string TensorRtBackend::backendName() const
{
    return "tensorrt";
}

bool TensorRtBackend::isLoaded() const
{
    return loaded_;
}

std::string TensorRtBackend::lastError() const
{
    return last_error_;
}

}  // namespace abu_yolo_ros
