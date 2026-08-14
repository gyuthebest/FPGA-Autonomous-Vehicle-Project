"""실캡처에서 신뢰도 로직의 정량 지표를 뽑는다.

BOARD_BRINGUP_RUNBOOK 8단계("정량 지표 산출")를 위한 도구다.
PASS/FAIL 만으로는 "신뢰도 및 위험도 기반"이라는 주장을 뒷받침할 수 없고,
다음 수치가 필요하다.

  - 채널별 상태 분포          정상 주행에서 DEGRADED/INVALID 로 보낸 표본 비율
                              (= 고장을 주입하지 않은 구간의 오탐률)
  - 검사기별 확정 비율        어떤 검사가 오탐을 만드는가
  - 관계식별 원시 위반 비율   디바운스 이전에 몇 %가 임계를 넘는가
  - 관계식별 잔차 분포        임계값이 양자화 바닥 위에 있는가

마지막 항목이 핵심이다.  기준값이 양자화 계단으로만 움직이면 잔차에는
"바닥"이 생기고, 임계값이 그 바닥보다 작으면 그 관계식은 정상 주행에서
구조적으로 통과할 수 없다.  이 경우 문제는 센서도 RTL도 아니고 임계값이다.

사용법
------
  python pl_capture_metrics.py <pl_capture_*.csv> [--skip 20] [--json out.json]
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys

from pl_model import (
    CHANNEL_ORDER, CONSISTENCY_RELATIONS, PLModel, RELATION_WIDTH, _trunc,
)
from compare_golden_vs_pl import decode_input_words


STATE_NAME = {0: "NORMAL", 1: "DEGRADED", 2: "INVALID"}
CHECKS = ("range_error", "jump_error", "stuck_error", "noise_error",
          "timeout_error")
CHECK_LABEL = {"range_error": "range", "jump_error": "jump",
               "stuck_error": "stuck", "noise_error": "noise",
               "timeout_error": "timeout"}


def _percentile(values, fraction: float):
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, int(len(ordered) * fraction))
    return ordered[index]


def load_samples(capture: Path):
    rows = []
    with capture.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            if row.get("fpga_response_valid", "").strip().lower() not in ("1", "true"):
                continue
            rows.append(row)
    return rows


def analyse(capture: Path, skip: int = 20) -> dict:
    rows = load_samples(capture)
    model = PLModel()

    states = {name: {0: 0, 1: 0, 2: 0} for name in CHANNEL_ORDER}
    checks = {name: {label: 0 for label in CHECKS} for name in CHANNEL_ORDER}
    cons_confirmed = {number: 0 for number, *_ in CONSISTENCY_RELATIONS}
    cons_raw_violation = {number: 0 for number, *_ in CONSISTENCY_RELATIONS}
    cons_active = {number: 0 for number, *_ in CONSISTENCY_RELATIONS}
    residuals = {number: [] for number, *_ in CONSISTENCY_RELATIONS}
    board_mismatch = {}
    compared = 0
    counted = 0

    for index, row in enumerate(rows):
        sample = decode_input_words([int(row[f"reg{i}_hex"], 16) for i in range(10)])
        # 마스크는 관계식 갱신 전 값이어야 하므로 step 이전에 읽는다.
        masks_before = model.rel._masks(
            sample, sample["situation"],
            model.pre.mask_1, model.pre.mask_2, model.pre.mask_3)
        out = model.step(sample, sample_seq=int(row["sample_seq"]),
                         situation=sample["situation"])
        if index < skip:
            continue
        counted += 1

        for name in CHANNEL_ORDER:
            states[name][out.state[name]] += 1
            for label in CHECKS:
                if getattr(model.rel.checks[name], label):
                    checks[name][label] += 1

        for number, channel, scale, threshold, pred_key, mask_key in CONSISTENCY_RELATIONS:
            width = RELATION_WIDTH[number]
            residual = abs(_trunc(sample.get(channel, 0), width) * scale
                           - _trunc(model.pre.pred.get(pred_key, 0), width))
            if out.cons_err[number]:
                cons_confirmed[number] += 1
            if not masks_before[mask_key]:
                cons_active[number] += 1
                residuals[number].append(residual)
                if residual > threshold:
                    cons_raw_violation[number] += 1

        word = int(row["fpga_reliability_word_hex"], 16)
        for position, name in enumerate(CHANNEL_ORDER):
            compared += 1
            if ((word >> (position * 2)) & 0x3) != out.state[name]:
                board_mismatch[name] = board_mismatch.get(name, 0) + 1

    return {
        "capture": capture.name,
        "samples_total": len(rows),
        "samples_counted": counted,
        "skip": skip,
        "states": states,
        "checks": checks,
        "relations": {
            number: {
                "channel": channel,
                "threshold": threshold,
                "active": cons_active[number],
                "raw_violation": cons_raw_violation[number],
                "confirmed": cons_confirmed[number],
                "residual_median": _percentile(residuals[number], 0.5),
                "residual_p90": _percentile(residuals[number], 0.9),
                "residual_max": max(residuals[number]) if residuals[number] else None,
            }
            for number, channel, _s, threshold, _p, _m in CONSISTENCY_RELATIONS
        },
        "board_compared": compared,
        "board_mismatch": board_mismatch,
    }


def report(result: dict) -> None:
    counted = result["samples_counted"] or 1
    print("=" * 78)
    print("실캡처 정량 지표")
    print("=" * 78)
    print(f"캡처   : {result['capture']}")
    print(f"표본   : {result['samples_total']} (예열 {result['skip']} 제외 -> {counted} 집계)")

    mismatch = sum(result["board_mismatch"].values())
    verdict = "일치" if mismatch == 0 else f"불일치 {mismatch}"
    print(f"보드 대조: {result['board_compared']}건 중 {verdict}")

    print("\n[1] 채널별 상태 분포  (고장 미주입 구간이므로 NORMAL 이외는 오탐)")
    print(f"  {'채널':16s} {'NORMAL':>9s} {'DEGRADED':>10s} {'INVALID':>9s}   오탐률")
    for name in CHANNEL_ORDER:
        counts = result["states"][name]
        bad = counts[1] + counts[2]
        print(f"  {name:16s} {counts[0]:9d} {counts[1]:10d} {counts[2]:9d}   "
              f"{100.0 * bad / counted:6.2f}%")

    print("\n[2] 검사기별 확정 비율")
    print(f"  {'채널':16s} " + " ".join(f"{CHECK_LABEL[c]:>8s}" for c in CHECKS))
    for name in CHANNEL_ORDER:
        row = result["checks"][name]
        if not any(row.values()):
            continue
        print(f"  {name:16s} " +
              " ".join(f"{100.0 * row[c] / counted:7.2f}%" for c in CHECKS))

    print("\n[3] 관계식별 판정  (active = 마스크되지 않은 표본)")
    print(f"  {'관계식':>6s} {'채널':16s} {'임계':>7s} {'active':>7s} "
          f"{'원시위반':>9s} {'확정':>8s} {'잔차중앙':>9s} {'잔차p90':>9s} {'잔차최대':>9s}")
    for number in sorted(result["relations"], key=int):
        item = result["relations"][number]
        active = item["active"] or 1
        def fmt(value):
            return "-" if value is None else f"{value:9d}"
        print(f"  {number:>6} {item['channel']:16s} {item['threshold']:7d} "
              f"{item['active']:7d} "
              f"{100.0 * item['raw_violation'] / active:8.1f}% "
              f"{100.0 * item['confirmed'] / counted:7.1f}% "
              f"{fmt(item['residual_median'])} {fmt(item['residual_p90'])} "
              f"{fmt(item['residual_max'])}")

    print("\n[4] 임계값 대비 잔차  (잔차 중앙값 > 임계값 이면 구조적으로 통과 불가)")
    for number in sorted(result["relations"], key=int):
        item = result["relations"][number]
        if not item["active"] or item["residual_median"] is None:
            continue
        if item["residual_median"] > item["threshold"]:
            print(f"  관계식 {number:>2} ({item['channel']}): "
                  f"잔차 중앙값 {item['residual_median']} > 임계 {item['threshold']} "
                  f"-> 정상 주행에서 통과 불가")


def main() -> int:
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except (AttributeError, OSError):
        pass
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--skip", type=int, default=20)
    parser.add_argument("--json", type=Path)
    args = parser.parse_args()

    if not args.capture.exists():
        sys.exit(f"캡처 파일이 없다: {args.capture}")

    result = analyse(args.capture, skip=args.skip)
    report(result)
    if args.json:
        args.json.write_text(json.dumps(result, indent=2, ensure_ascii=False),
                             encoding="utf-8")
        print(f"\nJSON: {args.json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
