"""Compare a CARLA PL capture with RTL replay output.

This checker deliberately does not duplicate the RTL decision equations.  It
checks sequence alignment, decodes diagnostic outputs, compares final commands
with the commands sent by CARLA, and evaluates externally visible safety
invariants.  Therefore a matching result is useful independently of the RTL
implementation.
"""

from __future__ import annotations

import argparse
import csv
from collections import Counter
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple


CHANNELS: Tuple[str, ...] = (
    "distance",
    "approach_speed",
    "accel_x",
    "accel_y",
    "accel_z",
    "gyro_x",
    "gyro_y",
    "gyro_z",
    "temperature",
    "humidity",
    "lux",
)

CHECK_COLUMNS: Tuple[str, ...] = (
    "range_mask",
    "jump_mask",
    "stuck_mask",
    "noise_mask",
    "consistency_mask",
    "timeout_mask",
)

STATE_NAMES = {0: "NORMAL", 1: "DEGRADED", 2: "INVALID", 3: "RESERVED"}


def _load_csv(path: Path) -> List[Dict[str, str]]:
    with path.open(newline="", encoding="utf-8-sig") as file_object:
        rows = list(csv.DictReader(file_object))
    if not rows:
        raise ValueError(f"CSV has no data rows: {path}")
    return rows


def _signed(value: int, bits: int) -> int:
    value &= (1 << bits) - 1
    sign = 1 << (bits - 1)
    return value - (1 << bits) if value & sign else value


def _command_fields(command: int) -> Dict[str, int]:
    return {
        "transition_demand": command & 1,
        "hud_warning": (command >> 1) & 1,
        "mrm": (command >> 2) & 1,
        "td_remain_sec": (command >> 3) & 0xF,
        "headlight": (command >> 7) & 1,
        "hazard": (command >> 8) & 1,
        "accelerator": (command >> 9) & 0xF,
        "brake": (command >> 13) & 0xF,
        "steering_raw": _signed((command >> 17) & 0xFF, 8),
        "gear": (command >> 25) & 0x3,
        "manual_mode": (command >> 27) & 1,
    }


def _capture_command_fields(row: Dict[str, str]) -> Dict[str, int]:
    # Compare against the exact values reconstructed by the PL preprocessor,
    # not the pre-quantization floating-point columns in the capture.
    reg6 = int(row["reg6_hex"], 16)
    reg7 = int(row["reg7_hex"], 16)
    reg8 = int(row["reg8_hex"], 16)
    return {
        "accelerator": (reg6 >> 26) & 0xF,
        "brake": (reg7 >> 24) & 0xF,
        # 조향은 reg7 상위 5비트 + reg8[26:24] 하위 3비트로 나뉘어 온다.
        "steering_raw": _signed(((reg7 >> 19) & 0x1F) << 3 | ((reg8 >> 24) & 0x7), 8),
        "gear": (reg7 >> 30) & 0x3,
        "manual_mode": reg8 & 1,
        "headlight": (reg8 >> 1) & 1,
        "hazard": (reg8 >> 2) & 1,
        "speed_limit": ((reg7 >> 11) & 0xFF) << 5,
    }


def _reliability_states(word: int) -> Dict[str, str]:
    return {
        channel: STATE_NAMES[(word >> (2 * index)) & 0x3]
        for index, channel in enumerate(CHANNELS)
    }


def _longest_true_run(values: Iterable[bool]) -> int:
    longest = 0
    current = 0
    for value in values:
        current = current + 1 if value else 0
        longest = max(longest, current)
    return longest


def compare(
    capture_path: Path,
    replay_path: Path,
    detail_path: Path,
    report_path: Path,
) -> None:
    capture_rows = _load_csv(capture_path)
    replay_rows = _load_csv(replay_path)
    capture_by_seq = {int(row["sample_seq"]): row for row in capture_rows}
    replay_by_seq = {int(row["sample_seq"]): row for row in replay_rows}

    capture_sequences = set(capture_by_seq)
    replay_sequences = set(replay_by_seq)
    missing_replay = sorted(capture_sequences - replay_sequences)
    unexpected_replay = sorted(replay_sequences - capture_sequences)
    common_sequences = sorted(capture_sequences & replay_sequences)

    reliability_counts = {channel: Counter() for channel in CHANNELS}
    detector_counts = {
        check: Counter({channel: 0 for channel in CHANNELS})
        for check in CHECK_COLUMNS
    }
    detector_runs: Dict[Tuple[str, str], List[bool]] = {
        (check, channel): [] for check in CHECK_COLUMNS for channel in CHANNELS
    }
    output_change_counts = Counter()
    flag_counts = Counter()
    invariant_counts = Counter()
    detail_rows: List[Dict[str, object]] = []

    first_flag_sequence: Dict[str, int] = {}
    last_flag_sequence: Dict[str, int] = {}

    for sequence in common_sequences:
        capture = capture_by_seq[sequence]
        replay = replay_by_seq[sequence]
        command = _command_fields(int(replay["command"], 16))
        source = _capture_command_fields(capture)
        reliability = _reliability_states(int(replay["reliability_word"], 16))
        speed_limit = int(replay["status_speed_limit"], 16) & 0x1FFF
        fault_label = capture.get("fault_label", "")

        for channel, state in reliability.items():
            reliability_counts[channel][state] += 1

        active_detectors: List[str] = []
        for check in CHECK_COLUMNS:
            mask = int(replay[check], 16)
            for bit, channel in enumerate(CHANNELS):
                active = bool((mask >> bit) & 1)
                detector_runs[(check, channel)].append(active)
                if active:
                    detector_counts[check][channel] += 1
                    active_detectors.append(f"{check.removesuffix('_mask')}:{channel}")

        changed_fields: List[str] = []
        for field in ("accelerator", "brake", "steering_raw"):
            if command[field] != source[field]:
                output_change_counts[field] += 1
                changed_fields.append(field)
        if speed_limit != source["speed_limit"]:
            output_change_counts["speed_limit"] += 1
            changed_fields.append("speed_limit")

        active_flags: List[str] = []
        for flag in ("hud_warning", "transition_demand", "mrm"):
            if command[flag]:
                flag_counts[flag] += 1
                active_flags.append(flag)
                first_flag_sequence.setdefault(flag, sequence)
                last_flag_sequence[flag] = sequence

        violations: List[str] = []
        if int(replay["risk_seq"]) != sequence:
            violations.append("risk_seq_mismatch")
        if int(replay["rel_seq"]) != sequence:
            violations.append("reliability_seq_mismatch")
        if command["mrm"] and not (
            command["accelerator"] == 0
            and command["brake"] == 3
            and command["hazard"] == 1
        ):
            violations.append("mrm_safe_command_violation")
        if command["transition_demand"] != int(command["td_remain_sec"] <= 10):
            violations.append("td_countdown_encoding_violation")
        if speed_limit > source["speed_limit"]:
            violations.append("speed_limit_increased")
        if source["manual_mode"] and not command["mrm"]:
            for field in ("accelerator", "brake", "steering_raw", "gear"):
                if command[field] != source[field]:
                    violations.append(f"manual_passthrough_{field}")

        for violation in violations:
            invariant_counts[violation] += 1

        nominal_detector_activity = fault_label in ("", "none") and bool(active_detectors)
        nominal_invalid = fault_label in ("", "none") and any(
            state == "INVALID" for state in reliability.values()
        )
        nominal_safety_intervention = fault_label in ("", "none") and bool(
            active_flags or command["accelerator"] != source["accelerator"]
            or command["brake"] != source["brake"]
        )

        if (
            violations
            or nominal_detector_activity
            or nominal_invalid
            or nominal_safety_intervention
        ):
            detail_rows.append({
                "sample_seq": sequence,
                "fault_label": fault_label,
                "reliability_word": replay["reliability_word"],
                "active_detectors": ";".join(active_detectors),
                "invalid_channels": ";".join(
                    channel for channel, state in reliability.items() if state == "INVALID"
                ),
                "active_control_flags": ";".join(active_flags),
                "changed_commands": ";".join(changed_fields),
                "input_accelerator": source["accelerator"],
                "output_accelerator": command["accelerator"],
                "input_brake": source["brake"],
                "output_brake": command["brake"],
                "input_steering_raw": source["steering_raw"],
                "output_steering_raw": command["steering_raw"],
                "input_speed_limit": source["speed_limit"],
                "output_speed_limit": speed_limit,
                "td_remain_sec": command["td_remain_sec"],
                "invariant_violations": ";".join(violations),
            })

    detail_path.parent.mkdir(parents=True, exist_ok=True)
    detail_fields = list(detail_rows[0]) if detail_rows else ["sample_seq"]
    with detail_path.open("w", newline="", encoding="utf-8-sig") as file_object:
        writer = csv.DictWriter(file_object, fieldnames=detail_fields)
        writer.writeheader()
        writer.writerows(detail_rows)

    nominal_count = sum(
        capture_by_seq[sequence].get("fault_label", "") in ("", "none")
        for sequence in common_sequences
    )
    invalid_nominal_count = 0
    for sequence in common_sequences:
        if capture_by_seq[sequence].get("fault_label", "") not in ("", "none"):
            continue
        states = _reliability_states(int(replay_by_seq[sequence]["reliability_word"], 16))
        invalid_nominal_count += any(state == "INVALID" for state in states.values())

    lines = [
        "# CARLA 캡처 대 PL RTL 재생 비교 보고서",
        "",
        "## 검증 범위",
        "",
        f"- CARLA 캡처: `{capture_path.resolve()}`",
        f"- RTL 재생 결과: `{replay_path.resolve()}`",
        f"- 캡처 샘플: {len(capture_rows)}개",
        f"- 재생 샘플: {len(replay_rows)}개",
        f"- 일치한 sample_seq: {len(common_sequences)}개",
        f"- 누락된 재생 sequence: {len(missing_replay)}개",
        f"- 예상하지 않은 재생 sequence: {len(unexpected_replay)}개",
        "",
        "이 보고서는 RTL 내부 식을 그대로 복제하지 않는다. 입력 라벨, 명령 입출력 관계, "
        "시퀀스 정합성 및 안전 불변조건을 독립적으로 검사한다.",
        "",
        "## 외부 안전 불변조건",
        "",
    ]
    if invariant_counts:
        lines.extend(
            f"- {name}: {count}건" for name, count in sorted(invariant_counts.items())
        )
    else:
        lines.append("- 위반 0건")

    lines.extend([
        "",
        "## 정상 라벨 데이터에서의 관찰",
        "",
        f"- `fault_label=none` 샘플: {nominal_count}개",
        f"- 하나 이상의 INVALID 채널이 나온 샘플: {invalid_nominal_count}개",
        f"- HUD 경고: {flag_counts['hud_warning']}개",
        f"- Transition Demand: {flag_counts['transition_demand']}개",
        f"- MRM: {flag_counts['mrm']}개",
        f"- 가속 명령 변경: {output_change_counts['accelerator']}개",
        f"- 제동 명령 변경: {output_change_counts['brake']}개",
        f"- 조향 명령 변경: {output_change_counts['steering_raw']}개",
        f"- 제한속도 변경: {output_change_counts['speed_limit']}개",
        "",
        "제어 개입 자체는 위험도 로직의 정상 동작일 수 있다. 하지만 이 캡처의 모든 샘플이 "
        "고장 없음으로 라벨링되어 있으므로 INVALID/TD/MRM은 오탐 후보로 분류하여 원인을 "
        "추가 확인해야 한다.",
        "",
        "## 신뢰도 상태 집계",
        "",
        "| 채널 | NORMAL | DEGRADED | INVALID | RESERVED |",
        "|---|---:|---:|---:|---:|",
    ])
    for channel in CHANNELS:
        counts = reliability_counts[channel]
        lines.append(
            f"| {channel} | {counts['NORMAL']} | {counts['DEGRADED']} | "
            f"{counts['INVALID']} | {counts['RESERVED']} |"
        )

    lines.extend([
        "",
        "## 검사기별 활성 샘플 수",
        "",
        "| 검사기 | 채널 | 활성 샘플 | 최장 연속 활성 |",
        "|---|---|---:|---:|",
    ])
    for check in CHECK_COLUMNS:
        for channel in CHANNELS:
            count = detector_counts[check][channel]
            if count:
                longest = _longest_true_run(detector_runs[(check, channel)])
                lines.append(f"| {check} | {channel} | {count} | {longest} |")

    lines.extend([
        "",
        "## 제어 플래그 최초/최종 sequence",
        "",
        "| 플래그 | 최초 | 최종 | 전체 활성 샘플 |",
        "|---|---:|---:|---:|",
    ])
    for flag in ("hud_warning", "transition_demand", "mrm"):
        lines.append(
            f"| {flag} | {first_flag_sequence.get(flag, '-')} | "
            f"{last_flag_sequence.get(flag, '-')} | {flag_counts[flag]} |"
        )

    lines.extend([
        "",
        "## 판정",
        "",
        "- AXI 전달 및 파이프라인 sequence 정합성은 외부 불변조건 기준으로 확인한다.",
        "- 정상 라벨 캡처에서 지속적으로 활성화된 검사기는 파라미터 또는 기준값 생성 방식의 "
        "  재검토 대상이다.",
        "- 세부 프레임은 별도 CSV에서 확인할 수 있다: "
        f"`{detail_path.resolve()}`",
        "",
    ])
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("capture", type=Path)
    parser.add_argument("replay", type=Path)
    parser.add_argument("--detail", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    compare(args.capture, args.replay, args.detail, args.report)
    print(args.report.resolve())
    print(args.detail.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
