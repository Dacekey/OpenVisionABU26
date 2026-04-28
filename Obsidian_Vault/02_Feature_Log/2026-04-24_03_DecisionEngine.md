# OpenVision – DecisionEngine Groundwork

**Date:** 2026-04-24  
**Status:** Stable groundwork later evolved into OpenVision-v3 legality interpretation  
**Workspace:** `~/openvision_ros2_ws_v2`  
**Package:** `abu_yolo_ros`

## 1. Historical Objective

This milestone originally introduced the DecisionEngine module during the older v2 phase.

At that time, the goal was to:

- combine YOLO semantic class information with TeamColorFilter output
- classify detections into a higher-level interpretation layer
- keep message contracts stable while the logic was still being tuned

## 2. Important Historical Implementation Work

Initial DecisionEngine work included:

- `include/abu_yolo_ros/decision_engine.hpp`
- `src/decision_engine.cpp`
- YAML-configurable thresholds and confidence weighting
- debug logging for semantic class, color result, confidence, and reasoning
- unit tests for important rule paths

An important bug was also corrected early:

- the logic was moved away from hard-coded `class_id` assumptions
- semantic `class_name` or family became the real source of truth

That design choice still matters in OpenVision-v3.

## 3. Terminology Update

The terminology has evolved since the original v2 wording.

Older action-oriented labels:

- `collect`
- `avoid`
- `unknown`

Current OpenVision-v3 vision-level labels:

- `legal`
- `illegal`
- `unknown`

Important clarification:

- the DecisionEngine now determines KFS legality interpretation
- it does **not** directly command robot action
- collection, skipping, route choice, target persistence, and servoing are downstream strategy or control decisions

## 4. How the DecisionEngine Evolved in OpenVision-v3

The original v2 DecisionEngine was symbol-level.

OpenVision-v3 now uses DecisionEngine logic in two contexts:

- **symbol-level DecisionEngine**
  retained mainly for compatibility, logging, and low-level debug interpretation
- **instance-level DecisionEngine**
  applied after `KFSInstanceAggregator` groups symbol detections into physical KFS candidates

The instance-level stage is now the more important runtime interpretation layer.

## 5. Current OpenVision-v3 Decision Logic

Current vision-level interpretation rules are centered on legality, not direct action.

High-level current rules:

- `FAKE` -> `illegal`
- `R1` -> `illegal`
- `REAL` with strong team-color match -> `legal`
- `REAL` without strong team-color match -> `unknown`
- ambiguous or unsupported evidence -> `unknown` or dropped depending on aggregation stage

Additional clarification:

- ambiguous mixed clusters may be dropped before final publication if the aggregation stage marks them unsafe
- low-confidence or weak-color cases should prefer `unknown` rather than an unsafe forced decision

## 6. Relationship Between Aggregation and Decision

In current OpenVision-v3, legality interpretation happens after KFS aggregation becomes available.

Current conceptual flow:

Camera/Image  
-> YOLO symbol detection  
-> symbol filtering  
-> KFSInstanceAggregator  
-> instance-level TeamColorFilter  
-> instance-level DecisionEngine  
-> `/yolo/kfs_instances`

This is a major architectural change from the older v2 symbol-only logic.

## 7. Current OpenVision-v3 Status

The DecisionEngine should now be understood as a legality classifier inside the KFS-level runtime.

Current role:

- interpret semantic KFS type
- combine that interpretation with team-color evidence
- produce `legal / illegal / unknown`
- remain separate from downstream strategy modules

The runtime no longer treats `collect / avoid` as the primary vision-level contract.

## 8. Relationship to Message Contracts

The modern runtime publishes structured KFS outputs through:

- `KfsInstance.msg`
- `KfsInstanceArray.msg`

Later milestones also added:

- `LocalizedKfsInstance.msg`
- `LocalizedKfsInstanceArray.msg`

This means the DecisionEngine is now part of a larger interpreted perception contract rather than a standalone debug-only classifier.

## 9. Relationship to Later Milestones

This groundwork later evolved into:

- `2026-04-25_04_KFSInstancePrototype.md`
- `2026-04-26_05_OpenVisionV3_KFSInstanceRuntime.md`
- `2026-04-27_06_OpenVisionV3_KFS3DLocalizer.md`
- `2026-04-27_07_OpenVisionV3_KFSLocalizationStabilizer.md`

Those later milestones moved the project from symbol-level interpretation toward:

- KFS-level instance reasoning
- legality-focused outputs
- localized and stabilized downstream contracts

## 10. Historical Notes Preserved

The following historical ideas from the v2 phase still remain valid:

- semantic labels are safer than relying on model index ordering
- thresholds should remain YAML-configurable
- debug logs are useful while tuning interpretation behavior
- low-confidence cases should default to conservative handling

What changed is not the need for a DecisionEngine, but the scope and terminology of what it now decides.

## 11. Validation History

Original validation from the groundwork phase remains relevant:

- `colcon build` passed
- semantic label handling was verified against non-linear `class_id` ordering
- threshold tuning and logic tests improved confidence before larger runtime integration

## 12. Final Interpretation

This file records the historical origin of the DecisionEngine, but the current OpenVision-v3 interpretation is:

- legality classification, not direct robot action
- symbol-level groundwork extended into instance-level KFS reasoning
- a supporting part of the larger KFS runtime pipeline rather than a standalone final decision layer
