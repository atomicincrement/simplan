#!/usr/bin/env python3
"""
F-16 model validation: compare our FDM against JSBSim reference.

Runs both simulations from identical initial conditions, parses their CSV
output, and prints a side-by-side comparison table with absolute differences.

Usage:
    python3 scripts/compare_f16.py

Requires:
    - jsbsim Python package  (pip install jsbsim)
    - Rust toolchain          (cargo must be on PATH)

Outputs:
    - scripts/output/our_f16.csv
    - scripts/output/jsbsim_f16.csv
    - Console comparison table
"""

import csv
import io
import math
import os
import subprocess
import sys

SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT   = os.path.dirname(SCRIPT_DIR)
OUTPUT_DIR  = os.path.join(SCRIPT_DIR, "output")
os.makedirs(OUTPUT_DIR, exist_ok=True)

OUR_CSV    = os.path.join(OUTPUT_DIR, "our_f16.csv")
JSB_CSV    = os.path.join(OUTPUT_DIR, "jsbsim_f16.csv")

# ── helpers ────────────────────────────────────────────────────────────────────

def run_and_save(cmd: list[str], csv_path: str, label: str) -> tuple[list[dict], list[str]]:
    """Run *cmd*, save stdout to *csv_path*, return (parsed_rows, info_lines).

    JSBSim prints a version banner to stdout before any data; we skip lines
    until we reach the CSV header (starts with "t_s").  Trim/info messages
    written to stderr are returned as *info_lines*.
    """
    print(f"  Running {label} ...", flush=True)
    result = subprocess.run(
        cmd,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    if result.returncode != 0:
        print(f"ERROR running {label}:", file=sys.stderr)
        print(result.stderr, file=sys.stderr)
        sys.exit(1)

    # Collect informational lines from stderr (e.g. JSBSim trim report).
    info_lines = [l.strip() for l in result.stderr.splitlines() if l.strip()]

    # Strip non-CSV preamble from stdout (JSBSim version banner etc.).
    lines = result.stdout.splitlines()
    csv_lines: list[str] = []
    in_csv = False
    for line in lines:
        if not in_csv:
            if line.startswith("t_s"):
                in_csv = True
                csv_lines.append(line)
        else:
            csv_lines.append(line)

    clean_csv = "\n".join(csv_lines)
    with open(csv_path, "w") as f:
        f.write(clean_csv + "\n")

    reader = csv.DictReader(io.StringIO(clean_csv))
    return list(reader), info_lines


def parse_float(v: str | None) -> float:
    try:
        return float(v)  # type: ignore[arg-type]
    except (ValueError, TypeError):
        return float("nan")


# ── run both simulations ───────────────────────────────────────────────────────

# Prefer the venv Python so jsbsim is available.
venv_python = os.path.join(REPO_ROOT, ".venv", "bin", "python3")
python_exe  = venv_python if os.path.exists(venv_python) else sys.executable

print("=" * 70)
print("F-16A model comparison: our FDM  vs  JSBSim reference")
print("=" * 70)

our_rows, our_info = run_and_save(
    ["cargo", "run", "--quiet", "-p", "fdm", "--example", "simulate"],
    OUR_CSV,
    "our FDM (Rust)",
)

jsb_rows, jsb_info = run_and_save(
    [python_exe, os.path.join(SCRIPT_DIR, "jsbsim_f16_test.py")],
    JSB_CSV,
    "JSBSim F-16",
)

# ── parse trim info from JSBSim stderr ────────────────────────────────────────
jsb_trim_alpha: float | None = None
jsb_trim_elev:  float | None = None
for line in jsb_info:
    if "alpha=" in line:
        try:
            jsb_trim_alpha = float(line.split("alpha=")[1].split()[0].rstrip("deg,"))
        except (IndexError, ValueError):
            pass
        try:
            jsb_trim_elev = float(line.split("elevator=")[1].split()[0].rstrip("deg,"))
        except (IndexError, ValueError):
            pass

# Our trim alpha is the alpha at t=0.
our_trim_alpha = parse_float(our_rows[0].get("alpha_deg")) if our_rows else float("nan")

# ── align rows by time ────────────────────────────────────────────────────────

jsb_by_t: dict[float, dict] = {}
for row in jsb_rows:
    t = round(parse_float(row.get("t_s")), 1)
    jsb_by_t[t] = row

# ── comparison table ──────────────────────────────────────────────────────────
# Table fits in 80 columns:
# t(5) + alt(6,6,+6) + vt(7,7,+6) + alpha(5,5,+5) + mach(6,6,+6) = ~80

print()
print("─" * 78)
print(f"  Trim alpha:  our FDM = {our_trim_alpha:.3f}°  |  "
      f"JSBSim = {jsb_trim_alpha:.3f}°  |  "
      f"Δ = {our_trim_alpha - jsb_trim_alpha:+.3f}°"
      if jsb_trim_alpha is not None
      else f"  Trim alpha:  our FDM = {our_trim_alpha:.3f}°")
if jsb_trim_elev is not None:
    print(f"  JSBSim trim elevator = {jsb_trim_elev:.3f}°  (our model: 0.0° open-loop, no FCS)")
print("  Note: JSBSim FCS maintains stable flight; our open-loop model diverges.")
print("─" * 78)
print(f"{'t':>4}  {'alt[ft]':>18}  {'vt[fps]':>20}  {'alpha[°]':>17}  {'Mach':>12}")
print(f"{'s':>4}  {'our':>6} {'JSB':>6} {'Δ':>5}  "
      f"{'our':>7} {'JSB':>7} {'Δ':>5}  "
      f"{'our':>5} {'JSB':>5} {'Δ':>5}  "
      f"{'our':>5} {'JSB':>5}")
print("─" * 78)

consecutive_invalid = 0
for our_row in our_rows:
    t = round(parse_float(our_row.get("t_s")), 1)
    jsb_row = jsb_by_t.get(t, {})

    ou_alt   = parse_float(our_row.get("alt_ft"))
    ou_vt    = parse_float(our_row.get("vt_fps"))
    ou_alpha = parse_float(our_row.get("alpha_deg"))
    ou_mach  = parse_float(our_row.get("mach"))
    js_alt   = parse_float(jsb_row.get("alt_ft"))
    js_vt    = parse_float(jsb_row.get("vt_fps"))
    js_alpha = parse_float(jsb_row.get("alpha_deg"))
    js_mach  = parse_float(jsb_row.get("mach"))

    # Detect rows where our FDM has blown up (abs values > physical limit).
    our_valid = (
        not math.isnan(ou_alt) and abs(ou_alt) < 200_000
        and not math.isnan(ou_vt) and abs(ou_vt) < 5_000
        and not math.isnan(ou_alpha) and abs(ou_alpha) < 90
    )
    if not our_valid:
        consecutive_invalid += 1
        if consecutive_invalid > 1:
            break      # stop printing after our FDM clearly diverged
        # still print the first bad row so the divergence is visible
        ou_alt = ou_vt = ou_alpha = ou_mach = float("nan")

    def fmt_trio(a: float, b: float, wa: int, wb: int, wd: int, dp: int) -> str:
        if math.isnan(a):
            return f"{'---':>{wa}} {'---':>{wb}} {'---':>{wd}}"
        if math.isnan(b):
            return f"{a:{wa}.{dp}f} {'---':>{wb}} {'---':>{wd}}"
        return f"{a:{wa}.{dp}f} {b:{wb}.{dp}f} {a-b:+{wd}.{dp}f}"

    def fmt_pair(a: float, b: float, w: int, dp: int) -> str:
        if math.isnan(a):
            return f"{'---':>{w}} {'---':>{w}}"
        if math.isnan(b):
            return f"{a:{w}.{dp}f} {'---':>{w}}"
        return f"{a:{w}.{dp}f} {b:{w}.{dp}f}"

    print(f"{t:4.0f}  {fmt_trio(ou_alt,js_alt,6,6,5,1)}  "
          f"{fmt_trio(ou_vt,js_vt,7,7,5,2)}  "
          f"{fmt_trio(ou_alpha,js_alpha,5,5,5,2)}  "
          f"{fmt_pair(ou_mach,js_mach,5,4)}")

print("─" * 78)
print()
print(f"CSVs written to:")
print(f"  {OUR_CSV}")
print(f"  {JSB_CSV}")
