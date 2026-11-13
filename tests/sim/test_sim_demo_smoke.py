"""Smoke test: headless sim_demo with 2 runs and reduced calibration poses. SIMULATION ONLY."""
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def test_sim_demo_headless_two_runs(tmp_path):
    out = tmp_path / "sim_demo"
    r = subprocess.run(
        [sys.executable, "-I", os.path.join(ROOT, "scripts", "sim_demo.py"), "--runs", "2", "--seed", "0",
         "--calib-poses", "6", "--board-poses", "6", "--out", str(out),
         "--kin-dir", str(tmp_path / "kin")],
        capture_output=True, text=True, timeout=900)
    assert r.returncode == 0, r.stdout[-2000:] + r.stderr[-2000:]
    s = (out / "summary.md").read_text(encoding="utf-8")
    assert "SIMULATION ONLY" in s
    rows = [ln for ln in s.splitlines() if re.match(r"\| \d+ \|", ln)]
    assert len(rows) == 2
    kin = sorted((tmp_path / "kin").glob("*.txt"))
    assert len(kin) == 2 and len(list((tmp_path / "kin").glob("*.csv"))) == 2
    assert (out / "runs.csv").exists()
    assert (out / "frames" / "run_01.png").exists() and (out / "frames" / "run_02.png").exists()
