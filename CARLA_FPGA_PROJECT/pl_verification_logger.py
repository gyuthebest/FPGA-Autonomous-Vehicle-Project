"""Lossless CARLA-to-PL verification capture.

This logger is deliberately independent from the dashboard CSV logger.  It
writes both a rich engineering capture and a compact REG0..REG9 vector file
that a SystemVerilog AXI replay testbench can consume without re-quantizing
Python floating-point values.
"""

from __future__ import annotations

import csv
from datetime import datetime
import os
from pathlib import Path
from typing import Optional, Sequence

from fpga_interface import FPGAResult, INPUT_WORD_COUNT


SCHEMA_VERSION = 2
DEFAULT_SAMPLE_RATE_HZ = 20.0


def _env_enabled(name: str, default: bool = True) -> bool:
    fallback = "1" if default else "0"
    return os.getenv(name, fallback).strip().lower() not in {
        "0", "false", "no", "off"
    }


class PLVerificationLogger:
    """Record the exact PL input image and the values that produced it."""

    def __init__(
        self,
        enabled: bool = True,
        output_dir: Optional[str] = None,
        sample_rate_hz: float = DEFAULT_SAMPLE_RATE_HZ,
        flush_every: int = 20,
    ) -> None:
        self.enabled = bool(enabled)
        self.sample_rate_hz = float(sample_rate_hz)
        self.nominal_period_ns = int(round(1_000_000_000.0 / self.sample_rate_hz))
        self.flush_every = max(1, int(flush_every))
        self._row_count = 0
        self._previous_send_ns: Optional[int] = None
        self.capture_path: Optional[Path] = None
        self.vector_path: Optional[Path] = None
        self._capture_file = None
        self._vector_file = None
        self._capture_writer = None
        self._vector_writer = None

        if not self.enabled:
            return

        base_dir = Path(__file__).resolve().parent
        log_dir = Path(output_dir) if output_dir else base_dir / "logs" / "pl_verification"
        log_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.capture_path = log_dir / f"pl_capture_{stamp}.csv"
        self.vector_path = log_dir / f"pl_vectors_{stamp}.csv"

        self._capture_file = self.capture_path.open("w", newline="", encoding="utf-8")
        self._vector_file = self.vector_path.open("w", newline="", encoding="ascii")
        self._capture_writer = csv.DictWriter(
            self._capture_file, fieldnames=self._capture_fields()
        )
        self._vector_writer = csv.writer(self._vector_file)
        self._capture_writer.writeheader()
        self._vector_writer.writerow(
            ["sample_seq", "host_gap_ns"] + [f"reg{i}" for i in range(INPUT_WORD_COUNT)]
        )

    @classmethod
    def from_environment(cls) -> "PLVerificationLogger":
        return cls(
            enabled=_env_enabled("PL_VERIFY_LOG", True),
            output_dir=os.getenv("PL_VERIFY_LOG_DIR") or None,
            sample_rate_hz=float(os.getenv("PL_VERIFY_SAMPLE_RATE_HZ", "20")),
            flush_every=int(os.getenv("PL_VERIFY_FLUSH_EVERY", "20")),
        )

    @staticmethod
    def _capture_fields():
        fields = [
            "schema_version", "sample_seq", "carla_frame", "simulation_time_s",
            "imu_frame", "imu_timestamp_s", "imu_frame_lag",
            "radar_frame", "radar_timestamp_s", "radar_frame_lag",
            "host_send_ns", "host_gap_ns", "host_gap_ms", "nominal_period_ns",
            "host_response_ns", "roundtrip_ns", "fault_label",
            "distance_m", "approach_speed_mps",
            "accel_x_mps2", "accel_y_mps2", "accel_z_mps2",
            "gyro_x_rps", "gyro_y_rps", "gyro_z_rps",
            "incline_x_deg", "incline_y_deg", "incline_z_deg",
            "speed_x_mps", "speed_y_mps", "speed_z_mps", "speed_kmh",
            "temperature_c", "humidity_pct", "lux", "weather",
            "requested_speed_limit_kmh", "rpm_level", "accelerator_cmd",
            "brake_cmd", "steering_normalized", "manual_mode", "gear",
            "headlight", "hazard", "situation",
        ]
        fields.extend(f"reg{i}_hex" for i in range(INPUT_WORD_COUNT))
        fields.extend([
            "fpga_response_valid", "fpga_sample_seq", "fpga_risk_word_hex",
            "fpga_reliability_word_hex", "fpga_accelerator", "fpga_brake",
            "fpga_steering_raw", "fpga_speed_limit_kmh", "fpga_headlight",
            "fpga_hazard", "fpga_transition_demand", "fpga_hud_warning",
            "fpga_mrm", "fpga_td_remain_sec",
        ])
        return fields

    def record(
        self,
        *,
        sample_seq: int,
        carla_frame: int,
        simulation_time_s: float,
        host_send_ns: int,
        host_response_ns: int,
        sensor,
        input_words: Sequence[int],
        requested_speed_limit_kmh: float,
        weather: int,
        rpm_level: int,
        accelerator_cmd: int,
        brake_cmd: int,
        steering_normalized: float,
        manual_mode: bool,
        gear: int,
        headlight: bool,
        hazard: bool,
        situation: int,
        fpga_result: Optional[FPGAResult] = None,
        fault_label: str = "none",
    ) -> None:
        if not self.enabled:
            return
        words = tuple(int(word) & 0xFFFFFFFF for word in input_words)
        if len(words) != INPUT_WORD_COUNT:
            raise ValueError(f"expected {INPUT_WORD_COUNT} PL words, got {len(words)}")

        send_ns = int(host_send_ns)
        response_ns = int(host_response_ns)
        gap_ns = 0 if self._previous_send_ns is None else send_ns - self._previous_send_ns
        self._previous_send_ns = send_ns

        row = {
            "schema_version": SCHEMA_VERSION,
            "sample_seq": int(sample_seq),
            "carla_frame": int(carla_frame),
            "simulation_time_s": f"{float(simulation_time_s):.9f}",
            "imu_frame": int(getattr(sensor, "imu_frame", -1)),
            "imu_timestamp_s": repr(float(getattr(sensor, "imu_timestamp", -1.0))),
            "imu_frame_lag": int(carla_frame) - int(getattr(sensor, "imu_frame", -1)),
            "radar_frame": int(getattr(sensor, "radar_frame", -1)),
            "radar_timestamp_s": repr(float(getattr(sensor, "radar_timestamp", -1.0))),
            "radar_frame_lag": int(carla_frame) - int(getattr(sensor, "radar_frame", -1)),
            "host_send_ns": send_ns,
            "host_gap_ns": gap_ns,
            "host_gap_ms": f"{gap_ns / 1_000_000.0:.6f}",
            "nominal_period_ns": self.nominal_period_ns,
            "host_response_ns": response_ns,
            "roundtrip_ns": max(0, response_ns - send_ns),
            "fault_label": str(fault_label),
            "distance_m": repr(float(sensor.distance)),
            "approach_speed_mps": repr(float(sensor.approach_speed)),
            "accel_x_mps2": repr(float(sensor.accel_x)),
            "accel_y_mps2": repr(float(sensor.accel_y)),
            "accel_z_mps2": repr(float(sensor.accel_z)),
            "gyro_x_rps": repr(float(sensor.gyro_x)),
            "gyro_y_rps": repr(float(sensor.gyro_y)),
            "gyro_z_rps": repr(float(sensor.gyro_z)),
            "incline_x_deg": repr(float(sensor.incline_x)),
            "incline_y_deg": repr(float(sensor.incline_y)),
            "incline_z_deg": repr(float(sensor.incline_z)),
            "speed_x_mps": repr(float(sensor.speed_x)),
            "speed_y_mps": repr(float(sensor.speed_y)),
            "speed_z_mps": repr(float(sensor.speed_z)),
            "speed_kmh": repr(float(sensor.speed)),
            "temperature_c": repr(float(sensor.temperature)),
            "humidity_pct": repr(float(sensor.humidity)),
            "lux": repr(float(sensor.lux)),
            "weather": int(weather),
            "requested_speed_limit_kmh": repr(float(requested_speed_limit_kmh)),
            "rpm_level": int(rpm_level),
            "accelerator_cmd": int(accelerator_cmd),
            "brake_cmd": int(brake_cmd),
            "steering_normalized": repr(float(steering_normalized)),
            "manual_mode": int(bool(manual_mode)),
            "gear": int(gear),
            "headlight": int(bool(headlight)),
            "hazard": int(bool(hazard)),
            "situation": int(situation),
            "fpga_response_valid": int(fpga_result is not None),
        }
        for index, word in enumerate(words):
            row[f"reg{index}_hex"] = f"{word:08X}"

        if fpga_result is not None:
            row.update({
                "fpga_sample_seq": fpga_result.sample_seq,
                "fpga_risk_word_hex": f"{fpga_result.risk_word:08X}",
                "fpga_reliability_word_hex": f"{fpga_result.reliability_word:08X}",
                "fpga_accelerator": fpga_result.accelerator,
                "fpga_brake": fpga_result.brake,
                "fpga_steering_raw": fpga_result.steering_raw,
                "fpga_speed_limit_kmh": fpga_result.speed_limit_kmh,
                "fpga_headlight": int(fpga_result.headlight),
                "fpga_hazard": int(fpga_result.hazard),
                "fpga_transition_demand": int(fpga_result.transition_demand),
                "fpga_hud_warning": int(fpga_result.hud_warning),
                "fpga_mrm": int(fpga_result.mrm),
                "fpga_td_remain_sec": fpga_result.td_remain_sec,
            })

        self._capture_writer.writerow(row)
        self._vector_writer.writerow(
            [int(sample_seq), gap_ns] + [f"{word:08X}" for word in words]
        )
        self._row_count += 1
        if self._row_count % self.flush_every == 0:
            self._capture_file.flush()
            self._vector_file.flush()

    def close(self) -> None:
        for file_object in (self._capture_file, self._vector_file):
            if file_object is not None and not file_object.closed:
                file_object.flush()
                file_object.close()
