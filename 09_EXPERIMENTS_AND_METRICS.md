# 09 --- Experiments and Metrics

## 1. Research question

Does adding robot-specific kinematic evaluation and
simulation-in-the-loop candidate validation improve manipulation
feasibility, success and/or efficiency compared with a simpler
vision-guided fixed-grasp baseline?

A second question is how accurately the digital twin predicts physical
motion and manipulation outcomes.

## 2. Experimental configurations

-   **A --- Baseline:** vision pose + one fixed grasp strategy +
    standard motion planning.
-   **B --- Kinematic selection:** vision pose + multiple grasp
    candidates + IK/joint-limit/reachability evaluation.
-   **C --- Simulation validation:** Configuration B plus Gazebo
    trajectory/collision validation.
-   **D --- Full system:** Configuration C plus supervisory task
    orchestration, verification and bounded recovery.

Keep the physical task and workspace conditions comparable between
configurations.

## 3. Independent variables

-   Cube position within the validated workspace.
-   Cube yaw/orientation.
-   Candidate selection configuration (A--D).
-   Lighting condition, if deliberately studied.
-   Trial number and repeat.

Do not introduce clutter, unknown objects or moving objects into the
primary Version 1 experiment.

## 4. Dependent metrics

  -----------------------------------------------------------------------
  Metric                              Definition
  ----------------------------------- -----------------------------------
  Detection rate                      Valid cube detections / image
                                      trials

  Position error                      Distance between estimated and
                                      reference cube position

  Orientation error                   Absolute yaw/orientation difference
                                      from reference

  IK success rate                     Candidate IK successes / candidates
                                      evaluated

  Planning success rate               Valid plans / planning requests

  Collision rejection rate            Candidates rejected for collision /
                                      candidates tested

  Grasp success rate                  Verified successful grasps /
                                      physical grasp attempts

  Placement success rate              Verified placements within
                                      tolerance / attempts

  Task success rate                   Fully successful pick-and-place
                                      tasks / trials

  Execution time                      Start of physical task to verified
                                      completion

  Planning time                       Candidate generation through
                                      selected plan

  Joint movement cost                 Defined joint-space path metric

  Recovery success rate               Recovered failures / eligible
                                      recoverable failures

  Sim-to-real gap                     Difference between predicted and
                                      measured outcomes
  -----------------------------------------------------------------------

Define exact reference measurement methods and tolerances before
collecting final data.

## 5. Trial design

Begin with a pilot set to identify failure modes and tune thresholds.
Then define the final trial count based on observed variability and the
time available. A starting target such as 30 trials per configuration
may be useful, but it is not automatically statistically sufficient.

Randomise or balance cube positions/yaws across configurations to reduce
order effects. Record every attempted trial, including failures and
aborted runs.

## 6. Logging schema

Each trial should record: - Trial ID and timestamp. - Software
commit/version. - Robot model/configuration and firmware where
available. - Camera settings and calibration ID. - Cube reference pose
and estimated pose. - Pose error. - Candidate list and rejection
reasons. - Selected candidate and score components. - IK and planning
result. - Simulation result. - Robot start/end joint state. - Gripper
command and feedback. - Execution timestamps. - Verification result. -
Recovery actions. - Stop/fault reason. - Operator notes.

Store raw images only where useful and permitted; use a clear naming
scheme and avoid overwriting trial records.

## 7. Reporting

Report: - Mean, median, standard deviation and confidence intervals
where appropriate. - Success rates with denominators. - Failure
categories. - Representative trajectories and pose-error plots. -
Simulation-versus-physical comparisons. - Limitations of the simulator
and perception setup.

Do not report only successful trials. Include failures and explain
exclusions.
