# KFS Target Priority Tracking, Closest KFS Selection, and Visual Servoing – Future Work

## 1. Overview

This note records three future-work ideas that are intentionally not implemented in the current OpenVision-v3 runtime.

Topics:

- KFS Target Priority Tracking
- Closest KFS Selection
- Visual Servoing

These are not required for the current perception milestone.

They may become useful later when the robot strategy layer, path planner, arm or gripper flow, and real Meihua navigation behavior are more clearly defined.

## 2. Current Decision

Current decision:

These features are postponed.

They should not be added to the main runtime yet.

Reason:

The current priority is to produce reliable KFS perception and localization data for:

- block-level Meihua path planning
- legal or illegal KFS identification
- future block map integration

The current OpenVision-v3 output should remain focused on:

- KFS instance detection
- legal, illegal, or unknown decision
- 3D localization
- stabilized robot-frame position

## 3. KFS Target Priority Tracking

Definition:

KFS Target Priority Tracking is a future strategy-level mechanism that keeps attention on a selected KFS target after a higher-level strategy module has decided that the KFS is worth collecting.

It may use:

- legal or illegal decision
- KFS position
- block ID
- route plan
- robot distance
- collection opportunity
- game phase
- arm or gripper availability

What it is not:

- not perception-level temporal smoothing
- not the current Kalman-based localization stabilizer
- not general object tracking for every KFS
- not required just to publish stable 3D position

Possible future purpose:

- remember the current collection target
- prevent the robot from switching targets every frame
- maintain target priority while the robot approaches
- support strategy-level collection sequencing
- help decide whether to continue, skip, or re-plan

Current status:

- not implemented
- kept as future strategy or planning work
- requires clearer decision from game strategy

## 4. Closest KFS Selection

Closest KFS Selection means selecting the nearest legal KFS candidate from known KFS instances.

Potential inputs:

- `/yolo/kfs_instances_stabilized`
- legal, illegal, or unknown decision
- robot-frame position
- `distance_mm`
- `bearing_deg`
- block ID when available
- path planner constraints
- game phase
- KFS availability or reachability

Important:

Closest does not always mean best.

Why:

- the nearest KFS may block an exit path
- the nearest KFS may be in a bad block
- the nearest KFS may be unreachable by the current route
- strategy may prefer escaping Meihua before collecting
- collection order may depend on block map and path plan

Because of that, Closest KFS Selection should not be implemented as a simple perception-layer rule yet.

Current status:

- not implemented
- future strategy or planning module should decide how to use distance and legality
- OpenVision-v3 should continue publishing enough data for this future selector

## 5. Visual Servoing

Visual Servoing is a future control-level technique for close-range alignment.

Purpose:

Use camera feedback to align the robot, arm, or gripper relative to a selected KFS target.

Possible use cases:

- fine alignment before grasping or pushing
- correcting small lateral error
- keeping the selected KFS centered in the image
- reducing final approach error
- supporting arm or gripper collection behavior

Potential inputs:

- selected KFS target
- bbox center error
- bottom-center projection
- 3D localized position
- bearing error
- arm or gripper feedback

Visual Servoing should be considered only after:

- the team has decided the collection strategy
- selected-target logic exists
- arm or gripper control flow is ready
- camera calibration and extrinsics are reliable enough

Current status:

- not implemented
- kept as future control or collection alignment work

## 6. Relationship to Current OpenVision-v3 Pipeline

Current relevant outputs:

- `/yolo/kfs_instances`
  KFS-level 2D perception
- `/yolo/kfs_instances_localized`
  raw 3D localized KFS
- `/yolo/kfs_instances_stabilized`
  stabilized robot-frame KFS position

These outputs provide the foundation for future:

- KFS Target Priority Tracking
- Closest KFS Selection
- Visual Servoing

But the current runtime should not make strategic collection decisions.

## 7. Why These Are Future Work

Reasons:

- current strategy prioritizes path finding and block-level KFS position awareness
- collection strategy is not finalized
- block map integration is not implemented yet
- real robot, Jetson, and camera calibration are not complete
- arm or gripper integration is not ready for visual servoing
- implementing these too early may produce the wrong architecture
- a simple closest-object rule may conflict with Meihua escape or path strategy

## 8. Possible Future Architecture

A possible future architecture:

`/yolo/kfs_instances_stabilized`  
-> BlockLocalizer / block map  
-> Legal KFS map  
-> Path planner / strategy module  
-> KFS Target Priority Selector  
-> optional Visual Servoing during approach or collection

KFS Target Priority Tracking should likely live in a strategy or planning package, not inside the low-level vision node.

Visual Servoing should likely live in a control or approach module, not inside the detector node.

## 9. Success Criteria If Implemented Later

For KFS Target Priority Tracking:

- stable target priority across frames
- does not switch target randomly
- respects legal or illegal status
- respects game phase and planner constraints

For Closest KFS Selection:

- selects closest legal or reachable KFS only when strategy allows collection
- can be disabled when path escape has priority
- uses stabilized 3D distance and block map information

For Visual Servoing:

- reduces final alignment error
- runs in realtime
- does not destabilize the main perception pipeline
- can be disabled safely
- works only after target selection is confirmed

## 10. Current Status

Current status summary:

- not implemented now
- added to Future Work
- current OpenVision-v3 will continue focusing on reliable perception, localization, stabilization, runtime safety, and backend performance
- these three mechanisms should be revisited after block map, path planner, and collection strategy are clearer

## 11. Future Work Checklist

- [ ] Finalize Meihua path planning strategy
- [ ] Define block map / `block_id` contract
- [ ] Define legal KFS map representation
- [ ] Decide when collection is allowed
- [ ] Design KFS Target Priority Tracking module
- [ ] Design Closest KFS Selection policy
- [ ] Integrate selected target with arm or gripper flow
- [ ] Evaluate Visual Servoing for close-range alignment
- [ ] Test on real robot after camera calibration and extrinsics are reliable

This note does not claim that these mechanisms are already implemented.

It also does not treat KFS Target Priority Tracking as perception-level temporal smoothing. That concept remains a future strategy-level target-selection mechanism and should only be revisited when higher-level planning and collection behavior are better defined.
