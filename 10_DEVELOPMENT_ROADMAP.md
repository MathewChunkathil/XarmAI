# 10 --- Development Roadmap

This is a proposed 16-week sequence. Adjust duration based on access to
the physical xArm, calibration progress and AG1010 model availability.

## Phase 0 --- Hardware and requirements confirmation

**Deliverables** - Confirm xArm 5 model, controller/firmware and
connection. - Confirm AG1010 interface, tool geometry and TCP. - Confirm
camera mounting and tabletop dimensions. - Freeze the initial cube and
marker design. - Confirm Ubuntu/ROS/Gazebo compatibility.

**Exit condition:** hardware facts and software versions are documented.

## Phase 1 --- Robot bring-up

**Deliverables** - Ubuntu 24.04 and ROS 2 Jazzy environment. - Official
UFACTORY driver running. - Joint-state and robot-status monitoring. -
AG1010 basic open/close control. - RViz robot visualisation. - MoveIt
plan-only test.

**Exit condition:** reliable read-only monitoring and supervised
low-risk robot tests.

## Phase 2 --- Simulation baseline

**Deliverables** - xArm 5 model in Gazebo. - AG1010 model or documented
approximation. - Table and cube collision geometry. - MoveIt/Gazebo
frame consistency. - Simulated safe motion.

**Exit condition:** repeatable simulated trajectory execution.

## Phase 3 --- Vision and calibration

**Deliverables** - C270 image acquisition. - AprilTag cube detection. -
Cube pose estimation. - Intrinsic calibration. - Camera-to-base
extrinsic calibration. - Save/load calibration files. - Pose validation
against measured references.

**Exit condition:** measured position/orientation error is documented
and acceptable for the chosen grasp strategy.

## Phase 4 --- Grasp and motion planning

**Deliverables** - Candidate generator for the known cube. -
Pre-grasp/approach/lift/placement poses. - IK and joint-limit
filtering. - Collision-aware MoveIt planning. - Candidate score and
rejection diagnostics.

**Exit condition:** candidate set is generated and ranked for multiple
cube poses.

## Phase 5 --- Digital-twin validation

**Deliverables** - Perception-to-Gazebo scene update. - Robot state
synchronisation. - Candidate trajectory simulation. - Simulation result
reporting. - Contact/grasp simulation if model fidelity allows.

**Exit condition:** simulation can reject invalid trajectories and
report its limitations.

## Phase 6 --- Supervised physical execution

**Deliverables** - Deterministic pre-execution gate. - Physical
pick-and-place sequence. - Grasp/lift/placement verification. - Bounded
recovery. - Operator stop handling.

**Exit condition:** supervised end-to-end trials complete and recorded.

## Phase 7 --- AI supervisor and dashboard

**Deliverables** - Structured task request. - Tool orchestration across
perception/planning/simulation. - Candidate explanation and failure
summaries. - Dashboard for camera, pose, robot state, candidate scores
and logs.

**Exit condition:** AI layer coordinates the pipeline without issuing
direct joint commands or bypassing gates.

## Phase 8 --- Evaluation and dissertation results

**Deliverables** - Configurations A--D benchmarked. - Repeated trials
and statistical summaries. - Simulation-to-real comparison. - Failure
analysis. - Final architecture, code and reproducibility documentation.

## Recommended priority

If schedule becomes constrained, protect the core scientific
contribution: 1. Reliable calibration and object pose. 2. xArm/AG1010
integration. 3. Robot-specific candidate and motion evaluation. 4.
Simulation validation. 5. Safe physical execution and measurable
results.

The dashboard and sophisticated AI interface should not delay these core
components.
