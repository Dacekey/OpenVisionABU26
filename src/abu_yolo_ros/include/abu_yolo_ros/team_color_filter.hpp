#pragma once

#include <string>

#include <opencv2/core.hpp>

#include "abu_yolo_ros/detection_types.hpp"

namespace abu_yolo_ros {

enum class TeamColor {
    RED,
    BLUE,
    UNKNOWN
};

struct TeamColorResult {
    bool matches_team;
    TeamColor detected_team;
    float confidence;
    float red_coverage;
    float blue_coverage;
    float mean_h;
    float mean_s;
    float mean_v;
    float dominant_coverage;
};

struct TeamColorFilterConfig {
    int red_h_low_1 = 0;
    int red_h_high_1 = 12;
    int red_h_low_2 = 168;
    int red_h_high_2 = 180;
    int red_s_low = 130;
    int red_v_low = 80;

    int blue_h_low = 100;
    int blue_h_high = 130;
    int blue_s_low = 150;
    int blue_v_low = 80;

    double min_coverage_ratio = 0.15;
    double confidence_scale = 3.0;
    double min_match_confidence = 0.30;
};

TeamColor parseTeamColor(const std::string& value);

std::string teamColorToString(TeamColor color);

TeamColorResult filterByTeamColor(
    const cv::Mat& bgr_image,
    const Detection& detection,
    TeamColor my_team,
    const TeamColorFilterConfig& config);

TeamColorResult filterByTeamColor(
    const cv::Mat& bgr_image,
    const cv::Rect& roi,
    TeamColor my_team,
    const TeamColorFilterConfig& config);

}  // namespace abu_yolo_ros
