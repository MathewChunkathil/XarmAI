# candidate_evaluator

Pure-Python grasp candidate generation, IK evaluation and ranking (IMPLEMENTATION_PLAN 5.6, 5.7, WP5, WP6.4).
No pybullet / ROS imports; ports to `/app/generate_candidates` and `/app/rank_candidates` later.

Layout: `src/candidate_evaluator/candidate_evaluator/` (add `src/candidate_evaluator` to `sys.path`).

## Inputs
- `cube_pose = (x, y, z, yaw)` cube centre in `link_base` (m, rad), `task_id`
- `config/gripper/gripper.yaml`, `config/planning/planning.yaml`, `config/workspace/workspace.yaml`, `config/planning/ranking.yaml` (units in comments; unmeasured values are tagged PLACEHOLDER)
- an `IKBackend` (`solve(pose_4x4, seed) -> (ok, q, reason)`, `joint_limits()`) and a seed joint vector

## Outputs
- `generate(...) -> list[GraspCandidate]` (fields as in 4.1 plus extras appended). Tool z points down, tool y is the closing axis. Geometric rejections: OUT_OF_WORKSPACE, GRIPPER_WIDTH, COLLISION (pregrasp below table / forbidden zone).
- `evaluate(candidates, backend, seed)` sets NO_IK / JOINT_LIMIT, stores `q_pregrasp/q_grasp/q_lift`, `joint_margin_min`.
- `rank(candidates, ranking_cfg, profile=None) -> RankResult(ranked, rejected, selected_id, ...)`. Profile A selects k=0 (bypass). Lowest total wins, ties by `candidate_id`. A term whose raw value is None (clearance, time before planning) is normalised to 0 and noted in `score.notes`.
- `format_table(candidates) -> str`.

## Test
```
.venv\Scripts\python tests\unit\test_candidates.py
.venv\Scripts\python tests\unit\test_rank.py
```
(or `python -m pytest tests/unit` if pytest is installed)
