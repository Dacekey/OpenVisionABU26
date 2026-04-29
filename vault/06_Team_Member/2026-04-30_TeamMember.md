# OpenVision-v3 Team Member and AI Collaboration Roles

## 1. Purpose

This document records the working roles and responsibilities used during the OpenVision-v3 development workflow.

- This is not an official organization chart.
- It is a practical project collaboration map.
- It helps clarify who or what contributed to system design, implementation, testing, and documentation.

## 2. Role Summary Table

| Member / Tool | Role Name | Main Responsibility |
|---|---|---|
| Dacekey | Project Owner / Vision System Architect | owns the OpenVisionABU26 direction, defines the robotics problem, decides strategy and architecture, tests the system in ROS 2, manages branches/commits/milestones, and reviews or accepts AI-generated changes |
| ChatGPT | AI Technical Advisor / System Design Co-Pilot | helps design the OpenVision-v3 architecture, explains ROS 2 / computer vision / AI concepts, proposes roadmap and task order, writes high-quality prompts for Codex/Gemini, reviews Codex outputs, helps decide between technical options, and keeps terminology/documentation consistent |
| Codex | AI Implementation Engineer | modifies code and documentation in the repository, reads actual source files, implements C++ / Python / YAML / Markdown tasks, runs build/test validation when available, and reports what changed and what passed or failed |
| Gemini | AI Documentation Assistant / Research Assistant | writes technical logs and Obsidian notes, helps summarize long technical milestones, assists with research notes such as camera information, and helps create readable documentation drafts |

## 3. Detailed Role Descriptions

### 3.1 Dacekey – Project Owner / Vision System Architect

Dacekey is the main decision maker for the project.

Responsibilities:

- defines the real robot and game problem
- decides OpenVision-v3 goals
- validates runtime behavior through ROS 2 tests
- reviews whether a feature is useful for Robocon strategy
- decides branch naming, commits, and milestones
- determines when a feature is accepted or postponed

Dacekey owns the final engineering judgment because the system must work on the real robot and fit the actual competition strategy.

### 3.2 ChatGPT – AI Technical Advisor / System Design Co-Pilot

ChatGPT helps translate goals into architecture and executable tasks.

Responsibilities:

- analyzes technical problems
- proposes perception pipeline design
- explains concepts such as QoS, circuit breaker, 3D localization, TensorRT, calibration, and related system behavior
- recommends task priority
- writes precise prompts for implementation agents
- reviews results from Codex
- helps separate implemented features from Future Work

ChatGPT does not directly modify the repository in this workflow. Its role is mainly architecture guidance, reasoning support, and task design.

### 3.3 Codex – AI Implementation Engineer

Codex is used as the implementation agent.

Responsibilities:

- edits repository files
- implements ROS 2 nodes, messages, config, and documentation updates
- inspects actual source code before making changes
- runs validation such as `colcon build`
- returns implementation summaries
- reports limitations or untested parts

Codex is responsible for repository-level implementation, but changes are still reviewed by Dacekey.

### 3.4 Gemini – AI Documentation Assistant / Research Assistant

Gemini is used mainly for documentation and research support.

Responsibilities:

- writes Markdown logs
- updates Obsidian documentation
- summarizes feature milestones
- helps research or organize external information
- prepares documentation drafts when needed

Gemini output should still be reviewed because technical logs must match the actual repository state and current system behavior.

## 4. Workflow Pattern

```text
Dacekey defines goal / issue / strategy
→ ChatGPT analyzes and designs task
→ ChatGPT writes prompt for Codex or Gemini
→ Codex implements code/docs or Gemini drafts documentation
→ Dacekey tests/reviews result
→ ChatGPT helps review and decide next step
→ Dacekey commits milestone
```

This loop keeps the project controlled by the human project owner while using AI tools to speed up implementation and documentation work.

## 5. Responsibility Boundaries

| Area | Primary Owner |
|---|---|
| Final architecture decision | Dacekey |
| Competition strategy | Dacekey |
| Technical advice / design alternatives | ChatGPT |
| Repository implementation | Codex |
| Documentation drafting | Gemini / Codex |
| Runtime testing | Dacekey |
| Code acceptance | Dacekey |
| Final technical report approval | Dacekey |

## 6. Notes

- AI tools are assistants, not final authority.
- All generated code and documentation should be reviewed.
- The real robot environment and competition rules should override assumptions from AI tools.
- Technical decisions should be validated with runtime tests whenever possible.

