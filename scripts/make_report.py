"""Build the PDF simulation report from per-run kinematics logs.  SIMULATION ONLY.

    .venv\\Scripts\\python -I scripts\\make_report.py [--logs results/kin_logs] [--since YYYYMMDD_HHMMSS] [--last N]
                                                      [--runs-csv results/sim_demo/runs.csv] [--out results/reports]
Writes <out>/<YYYYMMDD_HHMMSS>_xarm_sim_report.pdf and prints its path.
"""
import argparse
import csv
import glob
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from sim.report_pdf import build_report  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--logs", default=os.path.join(ROOT, "results", "kin_logs"))
    ap.add_argument("--since", default=None, help="only logs with file timestamp >= YYYYMMDD_HHMMSS")
    ap.add_argument("--last", type=int, default=None, help="only the newest N logs")
    ap.add_argument("--runs-csv", default=os.path.join(ROOT, "results", "sim_demo", "runs.csv"),
                    help="batch runs.csv (only used for a cross-check line in the console)")
    ap.add_argument("--out", default=os.path.join(ROOT, "results", "reports"))
    ap.add_argument("--png-dir", default=None, help="also save every page as PNG here (preview)")
    a = ap.parse_args()
    txts = sorted(glob.glob(os.path.join(a.logs, "*.txt")))
    if a.since:
        txts = [p for p in txts if os.path.basename(p)[:15] >= a.since]
    if a.last:
        txts = txts[-a.last:]
    if not txts:
        print("no kin logs found in %s" % a.logs, file=sys.stderr)
        return 1
    if os.path.exists(a.runs_csv):
        try:
            with open(a.runs_csv, newline="", encoding="utf-8") as f:
                rows = list(csv.DictReader(f))
            ok = sum(1 for r in rows if r.get("outcome") == "SUCCESS")
            print("runs.csv: %d runs, %d success (batch cross-check)" % (len(rows), ok))
        except Exception as e:
            print("runs.csv unreadable: %s" % e, file=sys.stderr)
    print("logs: %d" % len(txts))
    print(build_report(txts, a.out, png_dir=a.png_dir))
    return 0


if __name__ == "__main__":
    sys.exit(main())
