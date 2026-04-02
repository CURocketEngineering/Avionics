#!/usr/bin/env python3
"""
Plot GroundLevelEstimator CSV output.

CSV expected headers:
ts_ms,asl_m,egl_m,agl_m,launched

Usage:
  python plot_ground_level_csv.py
  python plot_ground_level_csv.py path/to/ground_level_test_output.csv
"""

from __future__ import annotations

import csv
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import matplotlib.pyplot as plt


@dataclass
class Row:
    t_s: float
    ts_ms: int
    asl_m: float
    egl_m: float
    agl_m: float
    launched: int


def _parse_int(name: str, v: str) -> int:
    try:
        return int(v)
    except ValueError as e:
        raise ValueError(f"Bad int for {name}: {v!r}") from e


def _parse_float(name: str, v: str) -> float:
    try:
        return float(v)
    except ValueError as e:
        raise ValueError(f"Bad float for {name}: {v!r}") from e


def read_csv(path: Path) -> List[Row]:
    if not path.exists():
        raise FileNotFoundError(f"CSV not found: {path}")

    rows: List[Row] = []
    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        required = {"ts_ms", "asl_m", "egl_m", "agl_m", "launched"}
        if reader.fieldnames is None:
            raise ValueError("CSV has no header row.")
        missing = required - set(reader.fieldnames)
        if missing:
            raise ValueError(f"CSV missing columns: {sorted(missing)}. Found: {reader.fieldnames}")

        for i, r in enumerate(reader, start=2):  # line number-ish (header is 1)
            ts_ms = _parse_int("ts_ms", r["ts_ms"])
            asl_m = _parse_float("asl_m", r["asl_m"])
            egl_m = _parse_float("egl_m", r["egl_m"])
            agl_m = _parse_float("agl_m", r["agl_m"])
            launched = _parse_int("launched", r["launched"])
            if launched not in (0, 1):
                raise ValueError(f"Line {i}: launched must be 0/1, got {launched!r}")

            rows.append(Row(t_s=ts_ms / 1000.0, ts_ms=ts_ms, asl_m=asl_m, egl_m=egl_m, agl_m=agl_m, launched=launched))

    if not rows:
        raise ValueError("CSV is empty (no data rows).")
    return rows


def first_launch_time(rows: List[Row]) -> Optional[float]:
    for r in rows:
        if r.launched == 1:
            return r.t_s
    return None


def plot(rows: List[Row], title: str) -> None:
    t = [r.t_s for r in rows]
    asl = [r.asl_m for r in rows]
    egl = [r.egl_m for r in rows]
    agl = [r.agl_m for r in rows]
    launched = [r.launched for r in rows]

    t_launch = first_launch_time(rows)

    # Figure 1: ASL + EGL
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111)
    ax1.plot(t, asl, label="ASL (m)")
    ax1.plot(t, egl, label="EGL (m)")
    ax1.set_title(f"{title} — ASL & EGL")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Meters")
    ax1.grid(True, which="both", linestyle=":")
    ax1.legend()

    # Figure 2: AGL + launch shading
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111)
    ax2.plot(t, agl, label="AGL (m)")
    ax2.set_title(f"{title} — AGL")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Meters")
    ax2.grid(True, which="both", linestyle=":")
    ax2.legend()

    if t_launch is not None:
        ax1.axvline(t_launch, linestyle="--", linewidth=1, label="Launch")
        ax2.axvline(t_launch, linestyle="--", linewidth=1, label="Launch")

        # Shade phases (pre-launch then launched)
        t0, tN = t[0], t[-1]
        ax1.axvspan(t0, t_launch, alpha=0.1)
        ax1.axvspan(t_launch, tN, alpha=0.05)
        ax2.axvspan(t0, t_launch, alpha=0.1)
        ax2.axvspan(t_launch, tN, alpha=0.05)

        # Rebuild legends to include the vline label cleanly
        ax1.legend()
        ax2.legend()

    # Figure 3: launched flag (step plot)
    fig3 = plt.figure()
    ax3 = fig3.add_subplot(111)
    ax3.step(t, launched, where="post", label="launched (0/1)")
    ax3.set_title(f"{title} — Launch Flag")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Flag")
    ax3.set_yticks([0, 1])
    ax3.grid(True, which="both", linestyle=":")
    ax3.legend()

    plt.show()


def main() -> int:
    default = Path("ground_level_test_output.csv")
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else default

    try:
        rows = read_csv(path)
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        return 2

    t_launch = first_launch_time(rows)
    agl_max = max(r.agl_m for r in rows)
    agl_min = min(r.agl_m for r in rows)

    # Basic sanity info
    print(f"Loaded: {path}")
    print(f"Rows: {len(rows)}")
    if t_launch is None:
        print("Launch: not detected in CSV (no launched==1 rows)")
    else:
        print(f"Launch: {t_launch:.3f} s (first launched==1)")
    print(f"AGL range: [{agl_min:.3f}, {agl_max:.3f}] m")

    plot(rows, title=path.name)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
