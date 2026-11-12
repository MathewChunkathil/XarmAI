# 05 --- Vision and Calibration

## 1. Version 1 vision objective

Use a fixed Logitech C270 to detect one known 50 mm AprilTag-marked cube
on a tabletop and estimate its position and yaw/orientation sufficiently
for the constrained grasp planner.

This is not a general object-recognition system. The marker family, IDs,
physical marker size, cube geometry and marker-to-cube transforms must
be known and configured.

## 2. Camera pipeline

``` mermaid
flowchart LR
    C[Logitech C270] --> IMG[Image acquisition]
    IMG --> PRE[Image validation / optional preprocessing]
    PRE --> TAG[AprilTag detection]
    TAG --> POSE[Cube pose estimation]
    POSE --> QC[Pose quality and uncertainty checks]
    QC --> TF[Transform to xArm base frame]
    TF --> OUT[Object pose + confidence + timestamp]
```

## 3. Required camera configuration

Record: - Camera device identifier. - Resolution and frame rate. - Pixel
format. - Exposure/focus settings if configurable. - Camera optical
frame name. - Image timestamp and frame sequence. - Calibration file
identifier.

Lock camera focus/exposure where practical after setup so image
characteristics remain stable. Validate actual C270 capabilities on the
target operating system rather than assuming all controls are available.

## 4. Calibration: two distinct parts

### 4.1 Intrinsic calibration

Estimate camera matrix and lens distortion: - Focal parameters. -
Principal point. - Distortion coefficients. - Image dimensions used for
calibration.

Use a practical printed calibration target or another validated
calibration method. A ChArUco board is not a Version 1 requirement.
Select a target that can be printed or otherwise accessed conveniently,
and validate reprojection error. A known AprilTag target may be usable
if the chosen calibration implementation supports it and the target
geometry is suitable.

### 4.2 Camera-to-robot extrinsic calibration

Estimate the rigid transform between the camera optical frame and xArm
base frame.

This requires a known reference relationship. Possible methods to
evaluate: - Observe a calibration target whose points are moved to or
measured at known robot-frame locations. - Use robot-guided point
correspondences with a suitable camera-visible target and a known robot
TCP. - Use a surveyed fixed reference with measured coordinates.

Choose one method and document its measurement procedure, assumptions
and error. Do not infer the transform from a single arbitrary cube
observation.

## 5. Fixed-camera Version 1 policy

-   Camera mounting position and orientation are fixed after
    commissioning.
-   Perform intrinsic and extrinsic calibration during initial setup.
-   Save the calibration parameters and metadata.
-   Load saved calibration automatically at startup.
-   Do not recalibrate during normal operation.
-   If the camera is physically moved, loosened or remounted, mark
    calibration as suspect and require manual recalibration.
-   Version 1 does not attempt automatic camera relocation detection.

Calibration metadata should include: - Camera identity and image
dimensions. - Camera matrix and distortion coefficients. -
Camera-to-base transform. - Frame names and transform convention. -
Calibration target/method. - Date, operator and software version. -
Reprojection/fit error and validation results. - Calibration schema
version and checksum.

## 6. Runtime pose estimation

For every detection: 1. Detect all configured marker IDs. 2. Reject
unknown IDs or inconsistent marker geometry. 3. Estimate marker poses.
4. Recover cube pose using known marker-to-cube transforms. 5. Check
reprojection error, pose consistency and visibility. 6. Estimate or
report pose uncertainty. 7. Transform pose to robot base frame. 8. Check
timestamp freshness and workspace bounds. 9. Publish only a valid pose;
otherwise publish a failure/invalid status.

## 7. Validation procedure

-   Collect images with cube at measured positions and several yaw
    angles.
-   Compare estimated pose with a measured reference.
-   Report position error in millimetres and orientation error in
    degrees.
-   Test partial marker visibility and different lighting.
-   Establish explicit rejection thresholds from measured data.
-   Verify transformed poses in RViz and against physical reference
    points.

## 8. Failure conditions

Do not plan a grasp if: - No valid marker is detected. - Marker IDs or
geometry are inconsistent. - Pose quality is below the configured
threshold. - Calibration is missing or invalid. - Image timestamp is
stale. - Cube pose lies outside the defined workspace. - Transform
lookup fails.
