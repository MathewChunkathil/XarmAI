# 08 --- Execution, Safety and Recovery

## 1. Non-negotiable principle

The AI supervisor must never directly command robot joints, bypass the
robot driver, or override safety checks. All physical movement passes
through a deterministic execution manager and a pre-execution gate.

The robot's native safety functions and physical emergency stop remain
primary. Application checks are additional safeguards, not replacements.

## 2. Pre-execution gate

A physical motion request is accepted only if: - Robot is connected and
in the expected mode. - No active robot fault or emergency-stop
condition exists. - Joint state is fresh and valid. - Gripper state is
known. - Calibration is loaded and marked valid. - Object pose is fresh
and inside the validated workspace. - Planning scene version matches the
plan. - Trajectory starts from the current or explicitly reconciled
robot state. - Joint limits and collision checks pass. -
Speed/acceleration settings are within approved limits. - Operator
supervision and physical stop access are available for commissioning.

Any failed check blocks motion and records a reason.

## 3. Execution stages

1.  Confirm preflight.
2.  Move to pre-grasp.
3.  Confirm motion completion and robot status.
4.  Approach at a conservative configured speed.
5.  Close AG1010 using the verified command interface.
6.  Check gripper feedback where available.
7.  Perform a small validated lift.
8.  Verify whether the cube moved with the gripper.
9.  Transfer only after grasp verification.
10. Place and release.
11. Verify the cube is at the target.
12. Retreat and record the result.

## 4. Verification

Use independent evidence where possible: - Gripper command and reported
position/state. - Robot motion completion/status. - Camera observation
before and after lift. - Cube pose relative to the gripper or expected
scene. - Cube pose after placement.

A gripper closing command alone is not proof of a successful grasp. A
camera image alone may also be ambiguous due to occlusion. Define a
confidence/status output: verified success, likely success, failure, or
uncertain.

## 5. Bounded recovery

Permitted Version 1 recovery examples: - Reacquire image if the cube
pose is stale. - Replan if the robot state changed before execution. -
Try the next ranked grasp candidate after a verified failed attempt, if
the cube remains in a known safe state. - Stop and request operator
intervention if the cube is displaced unpredictably, robot feedback is
missing, a collision is suspected, or a robot fault occurs.

Set a configurable maximum retry count. Do not create an unlimited retry
loop.

## 6. Mandatory stop conditions

-   Emergency stop or robot fault.
-   Unexpected robot state or tracking error.
-   Loss of communication.
-   Stale or invalid joint feedback.
-   Calibration invalidation.
-   Object pose uncertainty beyond the accepted threshold.
-   Unexpected collision or contact.
-   Gripper state inconsistent with the commanded action.
-   Object position becomes unknown.
-   Execution timeout.

On a stop condition: stop issuing new motion commands, invoke the
supported robot stop behaviour, preserve diagnostic data and request
operator intervention as appropriate.

## 7. Operating modes

-   `MANUAL`: operator controls robot using approved vendor interface;
    autonomy disabled.
-   `SIMULATION`: all motion confined to virtual robot.
-   `TWIN_MONITOR`: virtual robot mirrors measured physical robot state;
    no virtual command drives hardware.
-   `AUTONOMOUS_SUPERVISED`: task pipeline may execute after all gates
    pass and operator supervision is present.

Do not implement a mode that silently transitions from simulation to
physical execution.
