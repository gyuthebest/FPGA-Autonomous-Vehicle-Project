"""Analyze CARLA PL verification captures without changing RTL parameters."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import mean
from typing import Dict, Iterable, List, Sequence


NOMINAL_SAMPLE_RATE_HZ = 20.0
RANGE_CONFIRM_COUNT = 3
TIMEOUT_CONFIRM_COUNT = 10

# Current sensor_reliability.sv values, expressed in the integer units seen by PL.
RANGE_LIMITS = {
    "distance": (0, 20000),
    "approach_speed": (-4000, 4000),
    "accel_x": (-1600, 1600),
    "accel_y": (-1600, 1600),
    "accel_z": (-1600, 1600),
    "gyro_x": (-16000, 16000),
    "gyro_y": (-16000, 16000),
    "gyro_z": (-16000, 16000),
    "temperature": (-500, 600),
    "humidity": (0, 100),
    "lux": (0, 130000),
}

JUMP_THRESHOLDS = {
    "distance": 100,
    "approach_speed": 10,
    "accel_x": 2000,
    "accel_y": 2000,
    "accel_z": 2000,
    "gyro_x": 1000,
    "gyro_y": 1000,
    "gyro_z": 1000,
    "temperature": 5,
    "humidity": 5,
    "lux": 20000,
}


def signed(value: int, bits: int) -> int:
    value &= (1 << bits) - 1
    sign = 1 << (bits - 1)
    return value - (1 << bits) if value & sign else value


def decode_words(words: Sequence[int]) -> Dict[str, int]:
    if len(words) != 10:
        raise ValueError("a PL vector must contain REG0..REG9")
    r0, r1, r2, _r3, _r4, r5, r6, r7, _r8, _r9 = words
    return {
        "accel_x": signed(r0, 12),
        "accel_y": signed(r0 >> 16, 12),
        "accel_z": signed(r1, 12),
        "gyro_x": signed(r1 >> 16, 16),
        "gyro_y": signed(r2, 16),
        "gyro_z": signed(r2 >> 16, 16),
        "distance": r5 & 0x7FFF,
        "approach_speed": signed((r5 >> 15) & 0x3FF, 10) << 3,
        "humidity": (r5 >> 25) & 0x7F,
        "lux": r6 & 0x3FFFF,
        "temperature": signed(r7, 11),
    }


def percentile(values: Sequence[int], fraction: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return float(ordered[lower])
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def max_identical_run(values: Iterable[int]) -> int:
    longest = 0
    current = 0
    previous = object()
    for value in values:
        if value == previous:
            current += 1
        else:
            previous = value
            current = 1
        longest = max(longest, current)
    return longest


def load_capture(path: Path):
    with path.open(newline="", encoding="utf-8-sig") as file_object:
        rows = list(csv.DictReader(file_object))
    if not rows:
        raise ValueError(f"capture has no samples: {path}")
    decoded = []
    for row in rows:
        words = [int(row[f"reg{i}_hex"], 16) for i in range(10)]
        decoded.append(decode_words(words))
    return rows, decoded


def analyze(path: Path) -> Dict[str, object]:
    rows, decoded = load_capture(path)
    nominal_ns = int(rows[0].get("nominal_period_ns") or 1_000_000_000 / NOMINAL_SAMPLE_RATE_HZ)
    timeout_ns = nominal_ns * 2
    timeout_confirm_ns = timeout_ns * TIMEOUT_CONFIRM_COUNT
    gaps = [int(row["host_gap_ns"]) for row in rows[1:] if int(row["host_gap_ns"]) > 0]

    report: Dict[str, object] = {
        "capture": str(path.resolve()),
        "sample_count": len(rows),
        "first_sample_seq": int(rows[0]["sample_seq"]),
        "last_sample_seq": int(rows[-1]["sample_seq"]),
        "nominal_period_ms": nominal_ns / 1_000_000.0,
        "update_clk_x2_ms": timeout_ns / 1_000_000.0,
        "timeout_confirm_count": TIMEOUT_CONFIRM_COUNT,
        "timeout_confirm_ms": timeout_confirm_ns / 1_000_000.0,
        "host_gap_note": (
            "Host send gap is not the PS REG9 write gap. HIL validation must add PS-side timestamps."
        ),
    }
    if gaps:
        report["host_gap_ms"] = {
            "min": min(gaps) / 1_000_000.0,
            "mean": mean(gaps) / 1_000_000.0,
            "p50": percentile(gaps, 0.50) / 1_000_000.0,
            "p95": percentile(gaps, 0.95) / 1_000_000.0,
            "p99": percentile(gaps, 0.99) / 1_000_000.0,
            "max": max(gaps) / 1_000_000.0,
            "at_or_over_update_clk_x2": sum(gap >= timeout_ns for gap in gaps),
            "at_or_over_timeout_confirm": sum(
                gap >= timeout_confirm_ns for gap in gaps
            ),
        }

    channel_report = {}
    for channel, limits in RANGE_LIMITS.items():
        values = [sample[channel] for sample in decoded]
        deltas = [values[index] - values[index - 1] for index in range(1, len(values))]
        residuals = [deltas[index] - deltas[index - 1] for index in range(1, len(deltas))]
        out_of_range = [value < limits[0] or value > limits[1] for value in values]
        consecutive_range = 0
        confirmed_range_frames = 0
        for bad in out_of_range:
            consecutive_range = consecutive_range + 1 if bad else 0
            if consecutive_range >= RANGE_CONFIRM_COUNT:
                confirmed_range_frames += 1
        jump_threshold = JUMP_THRESHOLDS[channel]
        channel_report[channel] = {
            "pl_min": min(values),
            "pl_max": max(values),
            "range_limits": list(limits),
            "raw_out_of_range_samples": sum(out_of_range),
            "range_confirmed_frames_n3": confirmed_range_frames,
            "max_identical_run_samples": max_identical_run(values),
            "max_abs_delta": max((abs(value) for value in deltas), default=0),
            "max_abs_second_difference": max((abs(value) for value in residuals), default=0),
            "jump_threshold": jump_threshold,
            "jump_raw_hits": sum(abs(value) > jump_threshold for value in residuals),
        }
    report["channels"] = channel_report
    report["data_quality"] = {
        "sequence_contiguous": all(
            int(rows[index]["sample_seq"]) == int(rows[index - 1]["sample_seq"]) + 1
            for index in range(1, len(rows))
        ),
        "fault_labels": sorted({row.get("fault_label", "") for row in rows}),
    }
    if "imu_frame_lag" in rows[0]:
        imu_lags = [int(row["imu_frame_lag"]) for row in rows]
        radar_lags = [int(row["radar_frame_lag"]) for row in rows]
        report["data_quality"].update({
            "imu_frame_lag_min": min(imu_lags),
            "imu_frame_lag_max": max(imu_lags),
            "imu_not_same_frame_samples": sum(lag != 0 for lag in imu_lags),
            "radar_frame_lag_min": min(radar_lags),
            "radar_frame_lag_max": max(radar_lags),
            "radar_not_same_frame_samples": sum(lag != 0 for lag in radar_lags),
        })
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("capture", type=Path, help="pl_capture_*.csv")
    parser.add_argument("--output", type=Path, help="optional JSON report path")
    args = parser.parse_args()
    report = analyze(args.capture)
    rendered = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output:
        args.output.write_text(rendered + "\n", encoding="utf-8")
        print(args.output.resolve())
    else:
        print(rendered)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
