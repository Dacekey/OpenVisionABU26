#include "abu_yolo_ros/decision_engine.hpp"

#include <cmath>
#include <iostream>
#include <string>

namespace {

using abu_yolo_ros::DecisionEngineConfig;
using abu_yolo_ros::DecisionResult;
using abu_yolo_ros::KFSInstanceDecisionInput;
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
        r1_result.decision == KFSDecision::ILLEGAL,
        "class_name=R1 should always be ILLEGAL");
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
        real3_result.decision == KFSDecision::LEGAL,
        "REAL_3 with matching color and enough confidence should be LEGAL");
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
        real5_result.decision == KFSDecision::LEGAL,
        "REAL_5 with matching color and enough confidence should be LEGAL");

    const DecisionResult fake_result =
        abu_yolo_ros::classifyKFS(
            4,
            "FAKE_2",
            0.95,
            &matching_color,
            config);
    ok &= expect(
        fake_result.decision == KFSDecision::ILLEGAL,
        "FAKE_* should always be ILLEGAL");
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
        low_conf_avoid.decision == KFSDecision::ILLEGAL,
        "Low-confidence REAL should be ILLEGAL when unknown_on_low_confidence=false");

    const double expected_final_conf =
        config.yolo_confidence_weight * 0.92 +
        config.color_confidence_weight * matching_color.confidence;
    ok &= expect(
        approxEqual(real3_result.final_confidence, expected_final_conf),
        "REAL confidence fusion should remain unchanged");

    const KFSInstanceDecisionInput real_collect_input{
        "REAL",
        0.92,
        true,
        0.90,
        0.40,
        "normal",
        false,
        ""};
    const DecisionResult real_collect_result =
        abu_yolo_ros::classifyKFSInstance(
            real_collect_input,
            config);
    ok &= expect(
        real_collect_result.decision == KFSDecision::LEGAL,
        "REAL instance with team match and high confidence should be LEGAL");

    const KFSInstanceDecisionInput real_no_match_input{
        "REAL",
        0.92,
        false,
        0.90,
        0.40,
        "normal",
        false,
        ""};
    const DecisionResult real_no_match_result =
        abu_yolo_ros::classifyKFSInstance(
            real_no_match_input,
            config);
    ok &= expect(
        real_no_match_result.decision == KFSDecision::UNKNOWN,
        "REAL instance without team match should be UNKNOWN");

    const KFSInstanceDecisionInput real_low_conf_input{
        "REAL",
        0.40,
        true,
        0.90,
        0.40,
        "normal",
        false,
        ""};
    const DecisionResult real_low_conf_result =
        abu_yolo_ros::classifyKFSInstance(
            real_low_conf_input,
            config);
    ok &= expect(
        real_low_conf_result.decision == KFSDecision::UNKNOWN,
        "Low-confidence REAL instance should be UNKNOWN when unknown_on_low_confidence=true");

    const DecisionResult real_low_conf_avoid_result =
        abu_yolo_ros::classifyKFSInstance(
            real_low_conf_input,
            avoid_low_conf_config);
    ok &= expect(
        real_low_conf_avoid_result.decision == KFSDecision::ILLEGAL,
        "Low-confidence REAL instance should be ILLEGAL when unknown_on_low_confidence=false");

    const KFSInstanceDecisionInput fake_instance_input{
        "FAKE",
        0.95,
        false,
        0.0,
        0.10,
        "normal",
        false,
        ""};
    const DecisionResult fake_instance_result =
        abu_yolo_ros::classifyKFSInstance(
            fake_instance_input,
            config);
    ok &= expect(
        fake_instance_result.decision == KFSDecision::ILLEGAL,
        "FAKE instance should always be ILLEGAL");

    const KFSInstanceDecisionInput r1_instance_input{
        "R1",
        0.95,
        false,
        0.0,
        0.10,
        "normal",
        false,
        ""};
    const DecisionResult r1_instance_result =
        abu_yolo_ros::classifyKFSInstance(
            r1_instance_input,
            config);
    ok &= expect(
        r1_instance_result.decision == KFSDecision::ILLEGAL,
        "R1 instance should always be ILLEGAL");

    const KFSInstanceDecisionInput ambiguous_instance_input{
        "AMBIGUOUS",
        0.95,
        false,
        0.0,
        0.10,
        "normal",
        true,
        "too_many_symbols"};
    const DecisionResult ambiguous_instance_result =
        abu_yolo_ros::classifyKFSInstance(
            ambiguous_instance_input,
            config);
    ok &= expect(
        ambiguous_instance_result.decision == KFSDecision::UNKNOWN,
        "AMBIGUOUS instance should be UNKNOWN");

    const KFSInstanceDecisionInput unknown_group_input{
        "UNKNOWN",
        0.95,
        false,
        0.0,
        0.10,
        "normal",
        false,
        ""};
    const DecisionResult unknown_group_result =
        abu_yolo_ros::classifyKFSInstance(
            unknown_group_input,
            config);
    ok &= expect(
        unknown_group_result.decision == KFSDecision::UNKNOWN,
        "UNKNOWN group type should be UNKNOWN");

    return ok ? 0 : 1;
}
