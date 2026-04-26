#include "abu_yolo_ros/decision_engine.hpp"

#include <cmath>
#include <sstream>

namespace abu_yolo_ros {

namespace {

constexpr double kDefaultYoloWeight = 0.60;
constexpr double kDefaultColorWeight = 0.40;

double confidenceThresholdForClass(
    const std::string& class_name,
    const DecisionEngineConfig& config)
{
    if (isR1ClassName(class_name)) {
        return config.r1_conf_threshold;
    }

    if (isRealClassName(class_name)) {
        return config.real_conf_threshold;
    }

    return config.fake_conf_threshold;
}

bool isAmbiguousGroupType(const std::string& group_type)
{
    return group_type == "AMBIGUOUS";
}

bool isRealGroupType(const std::string& group_type)
{
    return group_type == "REAL";
}

bool isFakeGroupType(const std::string& group_type)
{
    return group_type == "FAKE";
}

bool isR1GroupType(const std::string& group_type)
{
    return group_type == "R1";
}

}  // namespace

std::string decisionToString(KFSDecision decision)
{
    switch (decision) {
    case KFSDecision::COLLECT:
        return "collect";
    case KFSDecision::AVOID:
        return "avoid";
    case KFSDecision::UNKNOWN:
    default:
        return "unknown";
    }
}

bool isR1ClassName(const std::string& class_name)
{
    return class_name == "R1";
}

bool isRealClassName(const std::string& class_name)
{
    return class_name.rfind("REAL_", 0) == 0;
}

bool isFakeClassName(const std::string& class_name)
{
    return class_name.rfind("FAKE_", 0) == 0;
}

DecisionEngineConfig normalizeDecisionConfig(
    const DecisionEngineConfig& config)
{
    DecisionEngineConfig normalized = config;
    const double weight_sum =
        normalized.yolo_confidence_weight +
        normalized.color_confidence_weight;

    if (weight_sum <= 0.0) {
        normalized.yolo_confidence_weight = kDefaultYoloWeight;
        normalized.color_confidence_weight = kDefaultColorWeight;
        return normalized;
    }

    normalized.yolo_confidence_weight /= weight_sum;
    normalized.color_confidence_weight /= weight_sum;
    return normalized;
}

DecisionResult classifyKFS(
    int class_id,
    const std::string& class_name,
    double yolo_confidence,
    const TeamColorResult* color_result,
    const DecisionEngineConfig& config)
{
    if (!isR1ClassName(class_name) &&
        !isRealClassName(class_name) &&
        !isFakeClassName(class_name)) {
        std::ostringstream reason;
        reason << "Invalid class_name '" << class_name
               << "' for class_id " << class_id;
        return {
            KFSDecision::AVOID,
            reason.str(),
            0.0};
    }

    const double required_threshold =
        confidenceThresholdForClass(class_name, config);

    if (yolo_confidence < required_threshold) {
        std::ostringstream reason;
        reason << "Low YOLO confidence "
               << yolo_confidence
               << " below threshold "
               << required_threshold;
        return {
            config.unknown_on_low_confidence ?
                KFSDecision::UNKNOWN :
                KFSDecision::AVOID,
            reason.str(),
            yolo_confidence};
    }

    if (isR1ClassName(class_name)) {
        return {
            KFSDecision::AVOID,
            "R1 KFS detected - AVOID",
            yolo_confidence};
    }

    if (isFakeClassName(class_name)) {
        return {
            KFSDecision::AVOID,
            "FAKE KFS detected - AVOID",
            yolo_confidence};
    }

    if (color_result == nullptr) {
        return {
            KFSDecision::AVOID,
            "REAL KFS but no team color result - AVOID",
            yolo_confidence * 0.5};
    }

    if (config.require_team_color_match &&
        !color_result->matches_team) {
        return {
            KFSDecision::AVOID,
            "REAL KFS but team color mismatch or unknown - AVOID",
            yolo_confidence};
    }

    const double final_confidence =
        config.yolo_confidence_weight * yolo_confidence +
        config.color_confidence_weight * color_result->confidence;

    if (final_confidence < config.collect_min_confidence) {
        return {
            KFSDecision::UNKNOWN,
            "REAL KFS with correct color but final confidence too low",
            final_confidence};
    }

    return {
        KFSDecision::COLLECT,
        "REAL KFS with correct team color - COLLECT",
        final_confidence};
}

DecisionResult classifyKFSInstance(
    const KFSInstanceDecisionInput& input,
    const DecisionEngineConfig& config)
{
    if (input.ambiguous || isAmbiguousGroupType(input.group_type)) {
        return {
            KFSDecision::UNKNOWN,
            "AMBIGUOUS KFS instance - UNKNOWN",
            0.0};
    }

    if (isFakeGroupType(input.group_type)) {
        return {
            KFSDecision::AVOID,
            "FAKE KFS instance - AVOID",
            input.symbol_confidence};
    }

    if (isR1GroupType(input.group_type)) {
        return {
            KFSDecision::AVOID,
            "R1 KFS instance - AVOID",
            input.symbol_confidence};
    }

    if (!isRealGroupType(input.group_type)) {
        return {
            KFSDecision::UNKNOWN,
            "Unknown KFS instance group type",
            0.0};
    }

    if (input.symbol_confidence < config.real_conf_threshold) {
        return {
            config.unknown_on_low_confidence ?
                KFSDecision::UNKNOWN :
                KFSDecision::AVOID,
            "Low REAL instance confidence",
            input.symbol_confidence};
    }

    if (config.require_team_color_match &&
        !input.team_color_match) {
        return {
            KFSDecision::UNKNOWN,
            "REAL KFS instance without team color match - UNKNOWN",
            input.symbol_confidence};
    }

    const double final_confidence =
        config.yolo_confidence_weight * input.symbol_confidence +
        config.color_confidence_weight * input.color_confidence;

    if (final_confidence < config.collect_min_confidence) {
        return {
            KFSDecision::UNKNOWN,
            "REAL KFS instance final confidence too low",
            final_confidence};
    }

    return {
        KFSDecision::COLLECT,
        "REAL KFS instance with correct team color - COLLECT",
        final_confidence};
}

}  // namespace abu_yolo_ros
