# Downstream State Handling Contract – legal / illegal / unknown / AMBIGUOUS

## 1. Purpose

This document records a future downstream design task: how planning and strategy modules should consume and respond to OpenVision-v3 KFS perception states.

Clarification:

- OpenVision-v3 currently detects and classifies KFS instances.
- It does not decide the final robot action.
- Downstream modules must decide whether to collect, ignore, avoid, re-check, or use the object for map or path-planning purposes.

## 2. Current Perception States

These states should be interpreted together, not independently.

### Decision states

- `legal`
- `illegal`
- `unknown`

### Group types

- `REAL`
- `FAKE`
- `R1`
- `AMBIGUOUS`
- `UNKNOWN`

### Localization states

- `localized=true/false`
- `localization_status`
- `localization_quality`
- `failure_reason`

For localized and stabilized outputs, the original perception state remains accessible through:

- `source_instance.group_type`
- `source_instance.decision`
- `source_instance.confidence`
- `source_instance.bbox`
- `source_instance.class_names`
- `source_instance.team_color`
- `source_instance.team_color_match`
- `source_instance.color_confidence`
- `source_instance.ambiguous`
- `source_instance.ambiguous_reason`

## 3. Meaning of `decision=legal`

`legal` means the vision system currently considers this KFS a valid candidate from perception and rule evidence.

Typical case:

- group type is `REAL`
- team-color evidence is strong enough
- confidence is sufficient
- instance is not ambiguous

Important:

`legal` does not mean the robot must collect immediately.

Downstream should still check:

- path strategy
- block map
- robot position
- reachability
- game phase
- collision or escape route
- arm or gripper availability

## 4. Meaning of `decision=illegal`

`illegal` means the KFS should not be considered a valid collection target from current perception and rule evidence.

Typical cases:

- group type is `FAKE`
- group type is `R1`
- class indicates a non-collectable or forbidden target
- current rule interpretation says it should not be collected

Suggested downstream usage:

- do not select it as a collection target
- optionally still record its position for path planning or obstacle/context awareness
- optionally use it to avoid wrong collection behavior

## 5. Meaning of `decision=unknown`

`unknown` means the perception system does not have enough reliable evidence to classify the instance as legal or illegal.

Possible causes:

- REAL-like symbol detected but team-color evidence is weak
- team-color result is uncertain
- low confidence
- missing or unreliable color mask
- ambiguous or incomplete visual evidence
- unsupported or unknown class group
- localization exists but semantic decision is uncertain

Suggested downstream handling:

- do not immediately collect
- keep it as a candidate requiring re-check
- use temporal observations if available
- wait for a better viewpoint or better frame
- treat it as uncertain in the block map
- avoid making irreversible decisions from a single unknown frame

## 6. Meaning of `group_type=AMBIGUOUS`

`AMBIGUOUS` means the KFS grouping itself is uncertain.

Current known ambiguous behavior:

- one major current case is when a cluster contains too many symbols for a normal KFS instance
- the current threshold is related to `max_symbols_per_instance`
- the current configured value is `3`
- if symbol count is greater than that threshold, the cluster may be considered ambiguous

Important distinction:

- ambiguous detection and ambiguous dropping are different
- if `drop_ambiguous_clusters=true`, ambiguous clusters may be removed from final output
- if `drop_ambiguous_clusters=false`, ambiguous clusters can remain visible with:
  - `ambiguous=true`
  - `ambiguous_reason`
  - `decision=unknown`

Current workspace note:

- the current YAML sets `drop_ambiguous_clusters=false`

Potential future improvement:

- mixed semantic groups such as `REAL + FAKE`, `REAL + R1`, or `FAKE + R1` should probably be handled carefully
- if current runtime does not mark all semantic mixing as ambiguous, downstream should still be cautious when `class_names` contain conflicting evidence

Suggested downstream handling:

- never treat `AMBIGUOUS` as `legal`
- do not collect based on `AMBIGUOUS` alone
- keep it for debug or map awareness if useful
- request re-observation from another frame or viewpoint
- prefer conservative strategy

## 7. Meaning of `group_type=UNKNOWN`

`UNKNOWN` means the semantic group type cannot be confidently classified.

Possible causes:

- class name not matching expected `REAL / FAKE / R1` patterns
- incomplete detection
- unsupported label
- unexpected model output
- aggregation could not assign a stable group type

Suggested downstream handling:

- treat it as uncertain
- do not collect
- keep only as low-confidence context if needed
- use repeated observations before taking action

## 8. Localization State Handling

### `localized=true`

This means a robot-frame position estimate is available.

Downstream can use:

- `position_robot_mm`
- `distance_mm`
- `bearing_deg`
- `plane_z_height_mm`
- `plane_height_source`

But downstream should still check semantic state before taking strategy action.

### `localized=false`

This means 3D localization failed.

Possible reasons include:

- `intersection_behind_camera`
- invalid bbox
- ray parallel to plane
- out of range
- calibration or extrinsics issue

Suggested downstream handling:

- do not use the position as a valid metric location
- rely on 2D info only if appropriate
- re-check after calibration or viewpoint changes
- avoid committing block-map placement from failed localization

## 9. Suggested Downstream Policy Matrix

| group_type | decision | localized | Suggested downstream handling |
|---|---|---:|---|
| REAL | legal | true | valid candidate for strategy/planning, not automatic collect |
| REAL | legal | false | semantic candidate, but needs localization before planning |
| REAL | unknown | any | re-check / uncertain candidate |
| FAKE | illegal | any | do not collect; may record as context |
| R1 | illegal | any | do not collect; may record as context |
| AMBIGUOUS | unknown | any | do not collect; re-observe/debug |
| UNKNOWN | unknown | any | do not collect; uncertain context only |

This matrix is only a starting point and should be revised after strategy and planning integration.

## 10. Relationship to Future Modules

### Block map / BlockLocalizer

Should use `legal / illegal / unknown` together with localization status to decide how to mark Meihua blocks.

### Path planning

Should not blindly route to the nearest KFS.

It should use:

- legal status
- block position
- Meihua escape strategy

### KFS Target Priority Tracking

Should only prioritize a KFS after strategy confirms it is useful and legal or reachable enough to matter.

### Closest KFS Selection

Should only consider legal or reachable candidates and should not override escape or path strategy.

### Visual Servoing

Should only run after a target is selected and confirmed by strategy.

## 11. Open Questions

- Should `unknown` be stored in the block map or ignored?
- Should `illegal` KFS still be stored as obstacle/context?
- Should `AMBIGUOUS` instances be published or dropped in production?
- Should mixed `REAL / FAKE / R1` clusters be explicitly marked `AMBIGUOUS`?
- How many frames are needed before downstream trusts a `legal` state?
- Should localization failure block all planning usage, or only collection usage?
- How should state confidence be combined with block-map confidence?

## 12. Current Status

- This is not implemented as a downstream module yet.
- OpenVision-v3 already publishes the required state information.
- Downstream modules must still define how to consume these states.
- This note is a design reminder for future strategy and planning integration.
