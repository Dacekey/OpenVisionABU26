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
    TeamColor my_team)
{
    TeamColorResult result{
        false,
        TeamColor::UNKNOWN,
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

    cv::Mat red_mask_1;
    cv::Mat red_mask_2;
    cv::Mat red_mask;
    cv::Mat blue_mask;

    cv::inRange(
        hsv_crop,
        cv::Scalar(0, 130, 80),
        cv::Scalar(12, 255, 255),
        red_mask_1);
    cv::inRange(
        hsv_crop,
        cv::Scalar(168, 130, 80),
        cv::Scalar(180, 255, 255),
        red_mask_2);
    cv::bitwise_or(red_mask_1, red_mask_2, red_mask);

    cv::inRange(
        hsv_crop,
        cv::Scalar(100, 150, 80),
        cv::Scalar(130, 255, 255),
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
    const float dominant_coverage =
        red_dominant ? result.red_coverage : result.blue_coverage;

    if (dominant_coverage >= kMinimumCoverage) {
        result.detected_team =
            red_dominant ? TeamColor::RED : TeamColor::BLUE;
        result.confidence = std::min(
            1.0f,
            dominant_coverage * kConfidenceScale);
    }

    result.matches_team =
        result.detected_team == my_team &&
        result.confidence >= kMatchThreshold;

    return result;
}

}  // namespace abu_yolo_ros
