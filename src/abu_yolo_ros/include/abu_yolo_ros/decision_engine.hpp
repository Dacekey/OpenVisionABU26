#pragma once

#include <string>

#include "abu_yolo_ros/team_color_filter.hpp"

namespace abu_yolo_ros {

enum class KFSDecision {
    COLLECT,
    AVOID,
    UNKNOWN
};

struct DecisionEngineConfig {
    double r1_conf_threshold = 0.55;
    double real_conf_threshold = 0.60;
    double fake_conf_threshold = 0.45;
    double collect_min_confidence = 0.60;

    double yolo_confidence_weight = 0.60;
    double color_confidence_weight = 0.40;

    bool require_team_color_match = true;
    bool unknown_on_low_confidence = true;
};

struct DecisionResult {
    KFSDecision decision;
    std::string reason;
    double final_confidence;
};

std::string decisionToString(KFSDecision decision);

bool isR1ClassName(const std::string& class_name);

bool isRealClassName(const std::string& class_name);

bool isFakeClassName(const std::string& class_name);

DecisionEngineConfig normalizeDecisionConfig(
    const DecisionEngineConfig& config);

DecisionResult classifyKFS(
    int class_id,
    const std::string& class_name,
    double yolo_confidence,
    const TeamColorResult* color_result,
    const DecisionEngineConfig& config);

}  // namespace abu_yolo_ros
