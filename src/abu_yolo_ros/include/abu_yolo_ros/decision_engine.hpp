#pragma once

#include <string>

#include "abu_yolo_ros/team_color_filter.hpp"

namespace abu_yolo_ros {

enum class KFSDecision {
    LEGAL,
    ILLEGAL,
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

struct KFSInstanceDecisionInput {
    std::string group_type;
    double symbol_confidence = 0.0;
    bool team_color_match = false;
    double color_confidence = 0.0;
    double color_mask_coverage = 0.0;
    std::string bbox_quality = "unknown";
    bool ambiguous = false;
    std::string ambiguous_reason;
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

DecisionResult classifyKFSInstance(
    const KFSInstanceDecisionInput& input,
    const DecisionEngineConfig& config);

}  // namespace abu_yolo_ros
