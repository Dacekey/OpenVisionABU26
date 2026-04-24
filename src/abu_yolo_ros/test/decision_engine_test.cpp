#include "abu_yolo_ros/decision_engine.hpp"

#include <cmath>
#include <iostream>
#include <string>

namespace {

using abu_yolo_ros::DecisionEngineConfig;
using abu_yolo_ros::DecisionResult;
using abu_yolo_ros::KFSDecision;
using abu_yolo_ros::TeamColor;
using abu_yolo_ros::TeamColorResult;

bool approxEqual(double left, double right, double epsilon = 1e-6)
{
    return std::fabs(left - right) <= epsilon;
}

bool expect(bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "[FAIL] " << message << std::endl;
        return false;
    }
    return true;
}

}  // namespace

int main()
{
    bool ok = true;

    const DecisionEngineConfig config{};
    const TeamColorResult matching_color{
        true,
        TeamColor::RED,
        0.90f,
        0.40f,
        0.02f,
        10.0f,
        200.0f,
        180.0f,
        0.40f};

    const DecisionResult r1_result =
        abu_yolo_ros::classifyKFS(
            15,
            "R1",
            0.90,
            &matching_color,
            config);
    ok &= expect(
        r1_result.decision == KFSDecision::AVOID,
        "class_name=R1 should always be AVOID");
    ok &= expect(
        r1_result.reason.find("R1 KFS detected") != std::string::npos,
        "R1 reason should mention R1 KFS");

    const DecisionResult real3_result =
        abu_yolo_ros::classifyKFS(
            24,
            "REAL_3",
            0.92,
            &matching_color,
            config);
    ok &= expect(
        real3_result.decision == KFSDecision::COLLECT,
        "REAL_3 with matching color and enough confidence should COLLECT");
    ok &= expect(
        real3_result.reason.find("REAL KFS with correct team color") != std::string::npos,
        "REAL_3 reason should mention correct team color");

    const DecisionResult real5_result =
        abu_yolo_ros::classifyKFS(
            26,
            "REAL_5",
            0.88,
            &matching_color,
            config);
    ok &= expect(
        real5_result.decision == KFSDecision::COLLECT,
        "REAL_5 with matching color and enough confidence should COLLECT");

    const DecisionResult fake_result =
        abu_yolo_ros::classifyKFS(
            4,
            "FAKE_2",
            0.95,
            &matching_color,
            config);
    ok &= expect(
        fake_result.decision == KFSDecision::AVOID,
        "FAKE_* should always be AVOID");
    ok &= expect(
        fake_result.reason.find("FAKE KFS detected") != std::string::npos,
        "FAKE reason should mention FAKE KFS");

    const DecisionResult low_conf_unknown =
        abu_yolo_ros::classifyKFS(
            24,
            "REAL_3",
            0.40,
            &matching_color,
            config);
    ok &= expect(
        low_conf_unknown.decision == KFSDecision::UNKNOWN,
        "Low-confidence REAL should be UNKNOWN when unknown_on_low_confidence=true");

    DecisionEngineConfig avoid_low_conf_config = config;
    avoid_low_conf_config.unknown_on_low_confidence = false;
    const DecisionResult low_conf_avoid =
        abu_yolo_ros::classifyKFS(
            24,
            "REAL_3",
            0.40,
            &matching_color,
            avoid_low_conf_config);
    ok &= expect(
        low_conf_avoid.decision == KFSDecision::AVOID,
        "Low-confidence REAL should be AVOID when unknown_on_low_confidence=false");

    const double expected_final_conf =
        config.yolo_confidence_weight * 0.92 +
        config.color_confidence_weight * matching_color.confidence;
    ok &= expect(
        approxEqual(real3_result.final_confidence, expected_final_conf),
        "REAL confidence fusion should remain unchanged");

    return ok ? 0 : 1;
}
