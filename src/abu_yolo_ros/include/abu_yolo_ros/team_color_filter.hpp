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
};

TeamColor parseTeamColor(const std::string& value);

std::string teamColorToString(TeamColor color);

TeamColorResult filterByTeamColor(
    const cv::Mat& bgr_image,
    const Detection& detection,
    TeamColor my_team);

}  // namespace abu_yolo_ros
