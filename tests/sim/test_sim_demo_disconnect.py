"""Headless test: simulated PyBullet disconnect after run 1 -> clean stop, partial results. SIMULATION ONLY."""
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def test_sim_demo_disconnect_partial(tmp_path):
    out = tmp_path / "sim_demo"
    env = dict(os.environ, XARM_SIM_TEST_DISCONNECT_AFTER_RUN="1")
    r = subprocess.run(
        [sys.executable, "-I", os.path.join(ROOT, "scripts", "sim_demo.py"), "--runs", "2", "--seed", "0",
         "--calib-poses", "6", "--board-poses", "6", "--out", str(out)],
        capture_output=True, text=True, timeout=900, env=env)
    assert r.returncode == 0, r.stdout[-2000:] + r.stderr[-2000:]
    assert "Stopped by user" in r.stdout
    s = (out / "summary_partial.md").read_text(encoding="utf-8")
    rows = [ln for ln in s.splitlines() if re.match(r"\| \d+ \|", ln)]
    assert len(rows) == 1
    assert "stopped by user after 1 runs" in s.lower()
    assert not (out / "summary.md").exists()
    assert "ABORTED_BY_USER" in (out / "runs_partial.csv").read_text()
