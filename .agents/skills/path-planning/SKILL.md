---
name: path-planning
description: Design, implement, review, and validate vehicle path/trajectory planning, including trajectory optimization, collision checking, lane transitions and parking paths. Use for wangliancar planning tasks; perception, behavior decisions, and actuator control remain separate modules.
license: MIT; see LICENSE.md
metadata:
  version: "1.0"
  category: automotive-engineering
  tags: adas, autonomous-driving, path-planning, trajectory-planning
  adaptation: wangliancar
  upstream-commit: feb68abe397acc14f32b34984975c48fedba2b33
---

# Path Planning Algorithm Skill

## Purpose
Enable path and trajectory planning for autonomous driving applications, including emergency and parking trajectories within the existing project interfaces. This is a project-adapted copy of the upstream skill; see [SOURCE.md](SOURCE.md) for provenance and changes.

Read the repository and planning module AGENTS instructions before applying this workflow. The repository instructions own responsibilities and permissions; the methods below do not authorize changing those boundaries.

## Capabilities
- Translate supplied behavior goals into trajectory constraints
- Trajectory optimization (polynomial, spline-based)
- Model-based constrained trajectory optimization when justified by the task
- Lattice planner implementation
- Collision checking algorithms
- Comfort and safety constraint handling
- Emergency maneuver planning
- Parking trajectory generation

## Usage Guidelines
- Consume behavior decisions and identify their required trajectory constraints
- Optimize trajectories for comfort and efficiency
- Implement robust collision checking at all planning stages
- Handle edge cases and emergency situations
- Validate planning algorithms in simulation
- Document algorithm parameters and tuning

## Dependencies

This installed skill is instruction-only: no scripts, packages, MCP tools, hooks, guards, or plugins are required. Use the repository's existing Python/SimOne environment and available inputs.

Upstream lists ROS/ROS2, Apollo, Autoware, and MATLAB/Simulink as dependencies. They are not requirements of this project adaptation. Do not install them (or Nav2) as part of using this skill. Evaluate candidate methods against the project's supported runtime rather than assuming a framework is available.

## Planning Workflow

1. Establish the scenario, provided decision, coordinate frame/reference point, units, time basis, data validity, and consumer expectations. Separate fields defined in a schema from fields actually populated by the producer.
2. Identify available reference paths, boundaries, obstacles and vehicle limits. Record missing inputs and interface needs; do not replace unknown geometry or limits with unverified constants.
3. Choose a method proportional to the required maneuver. Reference-path sampling, polynomial/spline optimization and lattice planning are alternatives, not mandatory dependencies or a fixed implementation sequence.
4. Check path direction, continuity, curvature and applicable vehicle constraints. Derive compatible speed and timing, handle zero speed explicitly, and assess obstacle clearance with vehicle footprint rather than only checking isolated points. State assumptions used for dynamic obstacles.
5. Define behavior for invalid inputs, route ends, unavailable paths and infeasible maneuvers through the agreed output contract. An invalid trajectory alone does not establish what the controller will do.
6. Validate using offline geometry/constraint cases, representative replay data when available, and separately authorized simulation. Distinguish path validity, trackability, and end-to-end scoring. Document parameters, evidence and limits; do not claim simulation results from unit tests.

For project-specific experience relevant to the task, consult [planning knowledge](../../../members/planning/KNOWLEDGE.md). Keep general methods in this skill and project evidence in that knowledge file.

## Process Integration

Upstream labels this skill ADA-002 (Path Planning and Motion Control), ADA-003 (ADAS Feature Development), and ADA-004 (Simulation and Virtual Validation). These are source taxonomy labels, not installed processes. No babysitter runtime or additional skills are needed; use this project's planning interface and test workflow.
