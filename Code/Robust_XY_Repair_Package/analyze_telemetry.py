#!/usr/bin/env python3
"""Summarize the one-line ESP32 telemetry format used by this controller.

Usage:
    python analyze_telemetry.py latest_hover_log.txt

The script never modifies the input file. It reports XY activation state,
position error, measured/fused velocity, attitude requests, yaw-rate extrema,
and the first few large-error samples.
"""

from __future__ import annotations

import argparse
import math
import re
from pathlib import Path
from typing import Dict, Iterable, List

FIELD_RE = re.compile(r"(?:^|,)([A-Z][A-Z0-9_]*):(-?\d+(?:\.\d+)?)")


def parse_rows(lines: Iterable[str]) -> List[Dict[str, float]]:
    rows: List[Dict[str, float]] = []
    for raw_line in lines:
        line = raw_line.strip()
        if not line.startswith("LOST:"):
            continue
        row: Dict[str, float] = {}
        for match in FIELD_RE.finditer(line):
            try:
                row[match.group(1)] = float(match.group(2))
            except ValueError:
                continue
        if row:
            rows.append(row)
    return rows


def finite_values(rows: Iterable[Dict[str, float]], key: str) -> List[float]:
    values: List[float] = []
    for row in rows:
        value = row.get(key)
        if value is not None and math.isfinite(value):
            values.append(value)
    return values


def max_abs(rows: Iterable[Dict[str, float]], key: str) -> float | None:
    values = finite_values(rows, key)
    return max((abs(value) for value in values), default=None)


def fmt(value: float | None, digits: int = 3) -> str:
    return "n/a" if value is None else f"{value:.{digits}f}"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path, help="controller telemetry text file")
    args = parser.parse_args()

    if not args.log.is_file():
        parser.error(f"file does not exist: {args.log}")

    rows = parse_rows(args.log.read_text(errors="replace").splitlines())
    if not rows:
        print("No telemetry lines beginning with LOST: were found.")
        return 2

    xy_rows = [row for row in rows if row.get("XY") == 1.0]
    print(f"telemetry_rows={len(rows)}")
    print(f"xy_active_rows={len(xy_rows)}")

    if not xy_rows:
        print("XY hold never reported active in this log.")
        return 0

    first = xy_rows[0]
    print("\nFirst XY-active sample")
    for key in ("ALT", "RNG", "Z", "ZT", "VZ", "X", "XT", "Y", "YT",
                "VX", "VY", "FVX", "FVY", "RAT", "PAT", "YR", "YC"):
        print(f"  {key}={fmt(first.get(key))}")

    x_errors = [abs(row["XT"] - row["X"]) for row in xy_rows
                if "XT" in row and "X" in row]
    y_errors = [abs(row["YT"] - row["Y"]) for row in xy_rows
                if "YT" in row and "Y" in row]

    print("\nExtrema while XY was active")
    print(f"  max_abs_x_position_error_m={fmt(max(x_errors, default=None))}")
    print(f"  max_abs_y_position_error_m={fmt(max(y_errors, default=None))}")
    for key in ("VX", "VY", "FVX", "FVY", "RAT", "PAT", "YR", "YC", "EIR"):
        print(f"  max_abs_{key}={fmt(max_abs(xy_rows, key))}")

    print("\nNotable samples")
    shown = 0
    for index, row in enumerate(xy_rows):
        x_error = abs(row.get("XT", 0.0) - row.get("X", 0.0))
        y_error = abs(row.get("YT", 0.0) - row.get("Y", 0.0))
        yaw_rate = abs(row.get("YR", 0.0))
        if x_error <= 0.50 and y_error <= 0.50 and yaw_rate <= 80.0:
            continue
        print(
            f"  xy_index={index} "
            f"ex={x_error:.3f} ey={y_error:.3f} "
            f"VX={row.get('VX', float('nan')):.3f} "
            f"VY={row.get('VY', float('nan')):.3f} "
            f"FVX={row.get('FVX', float('nan')):.3f} "
            f"FVY={row.get('FVY', float('nan')):.3f} "
            f"RAT={row.get('RAT', float('nan')):.2f} "
            f"PAT={row.get('PAT', float('nan')):.2f} "
            f"YR={row.get('YR', float('nan')):.2f} "
            f"YC={row.get('YC', float('nan')):.1f}"
        )
        shown += 1
        if shown >= 8:
            break

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
