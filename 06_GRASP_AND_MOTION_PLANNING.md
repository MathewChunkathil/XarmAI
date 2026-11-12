# 06 --- Grasp and Motion Planning

## 1. Purpose

Generate a limited set of grasp candidates for the known cube, reject
candidates that violate robot/tool/workspace constraints, plan complete
motions, and rank feasible candidates.

## 2. Version 1 grasp family

Start with simple, explainable grasps: - Top-down grasp. - Opposing
finger contact on two cube side faces. - Gripper closing axis aligned
with one of the cube's horizontal axes. - A small set of yaw
alternatives. - Configurable pre-grasp distance and lift height.

Do not begin with learned grasp networks or unconstrained 6-DoF grasp
generation. The known geometry and marker pose allow a deterministic
candidate generator.

## 3. Candidate record

Each candidate should contain: - Task ID and candidate ID. - Source cube
pose and timestamp. - Target grasp pose in robot base frame. - Pre-grasp
pose. - Approach direction and distance. - Gripper opening
command/target. - Expected contact faces. - Retreat/lift direction and
distance. - IK result and joint solution. - Planning result. - Collision
and clearance results. - Score components. - Rejection reason, if
rejected.

## 4. Hard constraints

Reject a candidate before ranking if any of these fail: - IK solution
unavailable. - Joint limit violation. - Pose outside validated
workspace. - Tool/cube collision in the pre-grasp or approach
configuration. - Collision with table or robot environment. - AG1010
opening incompatible with the grasp geometry. - Approach or retreat path
cannot be planned. - Required pose violates the actual 5-DoF kinematic
capability. - Scene or robot state is stale.

## 5. Full motion sequence

A complete pick-and-place plan contains: 1. Move to a known safe/ready
configuration. 2. Move to pre-grasp. 3. Approach the cube. 4. Close
gripper. 5. Lift vertically or along a validated safe direction. 6.
Transfer to the placement pre-position. 7. Approach placement pose. 8.
Open gripper. 9. Retreat. 10. Return to a safe/ready configuration.

Each segment must have a defined start/end state and success/failure
handling. The gripper action is part of the task sequence, not an
assumed instantaneous event.

## 6. Kinematic evaluation

Use the actual xArm 5 robot model and MoveIt configuration.

Evaluate: - IK success. - Joint-limit margin. - Distance from singular
or poorly conditioned configurations where measurable. - Manipulability
measure appropriate to the robot's 5-DoF task constraints. - Joint-space
path length. - Trajectory duration. - Collision clearance. - Feasibility
of the full approach and retreat path.

Do not apply a generic 6-DoF manipulability target without accounting
for the task's constrained orientation.

## 7. Candidate ranking

First apply hard constraints. Only feasible candidates are scored.

Example cost function:

`J_i = w_m C_motion + w_c C_clearance + w_k C_kinematic + w_g C_grasp + w_t C_time`

Where: - `C_motion`: normalised joint-space movement cost. -
`C_clearance`: penalty for low collision clearance. - `C_kinematic`:
penalty for poor joint-limit margin or poor conditioning. - `C_grasp`:
penalty for poor alignment/contact geometry. - `C_time`: predicted
trajectory duration.

All terms must be normalised before weighting. Store the individual
terms, weights and final score. Lower score can represent a better
candidate if consistently defined.

Initial weights are experimental parameters, not universal constants.
Establish a baseline and tune them using recorded trials. Never allow a
weighted score to override a hard safety or feasibility failure.

## 8. Planning interface

The grasp planner should request plans through a motion-planning
interface rather than controlling MoveIt internals directly. The
interface should return: - Success/failure. - Planned trajectory. -
Planning time. - Start state used. - Goal constraints. - Planning-scene
version. - Error code and diagnostic text.

## 9. Version 1 development sequence

1.  Plan to a manually specified safe pose.
2.  Plan to manually specified pre-grasp and grasp poses.
3.  Add candidate generation from a supplied cube pose.
4.  Add IK and joint-limit filtering.
5.  Add complete approach/lift/transfer/place/retreat plan.
6.  Add collision checks and candidate ranking.
7.  Connect the candidate evaluator to Gazebo validation.
8.  Connect the selected plan to supervised physical execution.
