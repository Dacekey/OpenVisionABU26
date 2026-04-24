#include "abu_yolo_ros/team_color_filter.hpp"

#include <algorithm>
#include <cctype>

#include <opencv2/imgproc.hpp>

namespace abu_yolo_ros {

namespace {

constexpr float kMinimumCoverage = 0.15f;
constexpr float kConfidenceScale = 3.0f;
constexpr float kMatchThreshold = 0.30f;

std::string normalizeTeamColorString(const std::string& value)
{
    std::string normalized;
    normalized.reserve(value.size());

    for (unsigned char ch : value) {
        if (!std::isspace(ch)) {
            normalized.push_back(
                static_cast<char>(std::tolower(ch)));
        }
    }

    return normalized;
}

cv::Rect clampDetectionToImage(
    const Detection& detection,
    const cv::Size& image_size)
{
    const int x1 = std::max(
        0,
        static_cast<int>(detection.x - detection.w / 2.0f));
    const int y1 = std::max(
        0,
        static_cast<int>(detection.y - detection.h / 2.0f));
    const int x2 = std::min(
        image_size.width,
        static_cast<int>(detection.x + detection.w / 2.0f));
    const int y2 = std::min(
        image_size.height,
        static_cast<int>(detection.y + detection.h / 2.0f));

    return cv::Rect(
        x1,
        y1,
        std::max(0, x2 - x1),
        std::max(0, y2 - y1));
}

}  // namespace

TeamColor parseTeamColor(const std::string& value)
{
    const std::string normalized =
        normalizeTeamColorString(value);

    if (normalized == "red") {
        return TeamColor::RED;
    }

    if (normalized == "blue") {
        return TeamColor::BLUE;
    }

    return TeamColor::UNKNOWN;
}

std::string teamColorToString(TeamColor color)
{
    switch (color) {
    case TeamColor::RED:
        return "red";
    case TeamColor::BLUE:
        return "blue";
    case TeamColor::UNKNOWN:
    default:
        return "unknown";
    }
}

TeamColorResult filterByTeamColor(
    const cv::Mat& bgr_image,
    const Detection& detection,
    TeamColor my_team,
    const TeamColorFilterConfig& config)
{
    TeamColorResult result{
        false,
        TeamColor::UNKNOWN,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f};

    if (bgr_image.empty()) {
        return result;
    }

    const cv::Rect roi =
        clampDetectionToImage(detection, bgr_image.size());

    if (roi.width <= 0 || roi.height <= 0) {
        return result;
    }

    const cv::Mat crop = bgr_image(roi);
    if (crop.empty()) {
        return result;
    }

    cv::Mat hsv_crop;
    cv::cvtColor(crop, hsv_crop, cv::COLOR_BGR2HSV);
    const cv::Scalar mean_hsv = cv::mean(hsv_crop);
    result.mean_h = static_cast<float>(mean_hsv[0]);
    result.mean_s = static_cast<float>(mean_hsv[1]);
    result.mean_v = static_cast<float>(mean_hsv[2]);

    cv::Mat red_mask_1;
    cv::Mat red_mask_2;
    cv::Mat red_mask;
    cv::Mat blue_mask;

    cv::inRange(
        hsv_crop,
        cv::Scalar(config.red_h_low_1, config.red_s_low, config.red_v_low),
        cv::Scalar(config.red_h_high_1, 255, 255),
        red_mask_1);
    cv::inRange(
        hsv_crop,
        cv::Scalar(config.red_h_low_2, config.red_s_low, config.red_v_low),
        cv::Scalar(config.red_h_high_2, 255, 255),
        red_mask_2);
    cv::bitwise_or(red_mask_1, red_mask_2, red_mask);

    cv::inRange(
        hsv_crop,
        cv::Scalar(config.blue_h_low, config.blue_s_low, config.blue_v_low),
        cv::Scalar(config.blue_h_high, 255, 255),
        blue_mask);

    const float crop_area = static_cast<float>(roi.area());
    if (crop_area <= 0.0f) {
        return result;
    }

    result.red_coverage =
        static_cast<float>(cv::countNonZero(red_mask)) / crop_area;
    result.blue_coverage =
        static_cast<float>(cv::countNonZero(blue_mask)) / crop_area;

    const bool red_dominant =
        result.red_coverage >= result.blue_coverage;
    result.dominant_coverage =
        red_dominant ? result.red_coverage : result.blue_coverage;

    if (result.dominant_coverage >= config.min_coverage_ratio) {
        result.detected_team =
            red_dominant ? TeamColor::RED : TeamColor::BLUE;
        result.confidence = std::min(
            1.0f,
            static_cast<float>(result.dominant_coverage * config.confidence_scale));
    }

    result.matches_team =
        result.detected_team == my_team &&
        result.confidence >= config.min_match_confidence;

    return result;
}

}  // namespace abu_yolo_ros
