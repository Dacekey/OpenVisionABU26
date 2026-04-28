#include "abu_yolo_ros/yolo_inference_utils.hpp"

#include <algorithm>
#include <sstream>
#include <stdexcept>

#include <opencv2/dnn.hpp>

namespace abu_yolo_ros {

namespace {

std::vector<int> applyNms(
    const std::vector<cv::Rect>& boxes,
    const std::vector<float>& scores,
    const float score_threshold,
    const float nms_threshold)
{
    std::vector<int> indices;
    cv::dnn::NMSBoxes(
        boxes,
        scores,
        score_threshold,
        nms_threshold,
        indices);
    return indices;
}

enum class OutputLayout {
    ChannelsFirst,
    PredictionsFirst
};

struct OutputLayoutInfo {
    OutputLayout layout = OutputLayout::ChannelsFirst;
    int num_predictions = 0;
    int num_channels = 0;
};

OutputLayoutInfo parseOutputLayout(
    const std::vector<int64_t>& output_shape)
{
    if (output_shape.size() < 2 || output_shape.size() > 3) {
        std::ostringstream stream;
        stream << "Unsupported YOLO output rank: " << output_shape.size();
        throw std::runtime_error(stream.str());
    }

    const int64_t dim_a = output_shape[output_shape.size() - 2];
    const int64_t dim_b = output_shape[output_shape.size() - 1];
    if (dim_a <= 0 || dim_b <= 0) {
        throw std::runtime_error("YOLO output shape contains non-positive dimensions");
    }

    if (dim_a == dim_b) {
        std::ostringstream stream;
        stream << "Ambiguous YOLO output layout with shape [";
        for (std::size_t i = 0; i < output_shape.size(); ++i) {
            if (i > 0) {
                stream << ", ";
            }
            stream << output_shape[i];
        }
        stream << "]";
        throw std::runtime_error(stream.str());
    }

    OutputLayoutInfo info;
    if (dim_a < dim_b) {
        info.layout = OutputLayout::ChannelsFirst;
        info.num_channels = static_cast<int>(dim_a);
        info.num_predictions = static_cast<int>(dim_b);
    } else {
        info.layout = OutputLayout::PredictionsFirst;
        info.num_predictions = static_cast<int>(dim_a);
        info.num_channels = static_cast<int>(dim_b);
    }

    if (info.num_channels <= 4) {
        std::ostringstream stream;
        stream << "Unsupported YOLO output channel count: "
               << info.num_channels;
        throw std::runtime_error(stream.str());
    }

    return info;
}

float readOutputValue(
    const float* output_data,
    const OutputLayoutInfo& layout_info,
    const int channel,
    const int prediction)
{
    if (layout_info.layout == OutputLayout::ChannelsFirst) {
        return output_data[channel * layout_info.num_predictions + prediction];
    }
    return output_data[prediction * layout_info.num_channels + channel];
}

}  // namespace

std::vector<float> preprocessImageToChw(
    const cv::Mat& image,
    const int input_width,
    const int input_height)
{
    cv::Mat resized;
    cv::resize(image, resized, cv::Size(input_width, input_height));

    cv::Mat rgb;
    cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
    rgb.convertTo(rgb, CV_32F, 1.0 / 255.0);

    std::vector<float> input_tensor_values(
        static_cast<std::size_t>(3 * input_width * input_height));
    for (int c = 0; c < 3; ++c) {
        for (int h = 0; h < input_height; ++h) {
            for (int w = 0; w < input_width; ++w) {
                input_tensor_values[static_cast<std::size_t>(
                    c * input_width * input_height + h * input_width + w)] =
                    rgb.at<cv::Vec3f>(h, w)[c];
            }
        }
    }

    return input_tensor_values;
}

std::vector<Detection> decodeYoloDetections(
    const float* output_data,
    const std::vector<int64_t>& output_shape,
    const cv::Size& original_image_size,
    const InferenceBackendConfig& config,
    const int input_width,
    const int input_height)
{
    if (!output_data) {
        throw std::runtime_error("YOLO output tensor is null");
    }

    const OutputLayoutInfo layout_info = parseOutputLayout(output_shape);
    const int num_classes = layout_info.num_channels - 4;

    const float scale_x =
        static_cast<float>(original_image_size.width) /
        static_cast<float>(input_width);
    const float scale_y =
        static_cast<float>(original_image_size.height) /
        static_cast<float>(input_height);

    std::vector<Detection> raw_detections;
    std::vector<cv::Rect> boxes;
    std::vector<float> scores;

    for (int i = 0; i < layout_info.num_predictions; ++i) {
        const float x = readOutputValue(output_data, layout_info, 0, i);
        const float y = readOutputValue(output_data, layout_info, 1, i);
        const float w = readOutputValue(output_data, layout_info, 2, i);
        const float h = readOutputValue(output_data, layout_info, 3, i);

        int best_class = -1;
        float best_score = 0.0F;
        for (int c = 0; c < num_classes; ++c) {
            const float score =
                readOutputValue(output_data, layout_info, 4 + c, i);
            if (score > best_score) {
                best_score = score;
                best_class = c;
            }
        }

        if (best_score <= config.conf_threshold) {
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

        boxes.emplace_back(
            static_cast<int>(det.x - det.w * 0.5F),
            static_cast<int>(det.y - det.h * 0.5F),
            static_cast<int>(det.w),
            static_cast<int>(det.h));
        scores.push_back(det.confidence);
    }

    const std::vector<int> keep_indices = applyNms(
        boxes,
        scores,
        config.conf_threshold,
        config.nms_threshold);

    std::vector<Detection> final_detections;
    final_detections.reserve(keep_indices.size());
    for (const int idx : keep_indices) {
        final_detections.push_back(raw_detections[idx]);
    }

    return final_detections;
}

}  // namespace abu_yolo_ros
