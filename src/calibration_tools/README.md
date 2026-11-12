# calibration_tools
- `kabsch.umeyama / umeyama_ransac / rigid_error`: point-set rigid fit (P,Q Nx3 -> R,t,s); errors in mm / deg.
- `handeye_A.calibrate_eye_to_hand(T_base_tcp_list, T_cam_tag_list)`: solves T_base_cam and T_tcp_tag jointly (A X = Y B); no ground truth.
- `intrinsics.calibrate_checkerboard(images, pattern, square_m)` -> K, dist, rms.
No simulator dependencies.
