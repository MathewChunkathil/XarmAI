"""PDF report builder test (uses existing kin logs). SIMULATION ONLY."""
import glob
import os
import re
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
pytest.importorskip("matplotlib")
from sim.report_pdf import build_report  # noqa: E402


def test_build_report_from_two_logs(tmp_path):
    logs = sorted(glob.glob(os.path.join(ROOT, "results", "kin_logs", "*.txt")))[:2]
    if len(logs) < 2:
        pytest.skip("no kin logs available")
    bad = tmp_path / "broken.txt"
    bad.write_text("garbage")
    out = build_report(logs + [str(bad)], str(tmp_path))
    assert os.path.exists(out) and out.endswith("_xarm_sim_report.pdf")
    data = open(out, "rb").read()
    assert len(data) > 10 * 1024
    assert len(re.findall(rb"/Type\s*/Page(?![s\w])", data)) >= 4
