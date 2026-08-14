"""고장 주입 구간에서 검출 지연 / 복구 지연 / 미검출을 산출한다.

BOARD_BRINGUP_RUNBOOK 8단계의 나머지 절반이다.
`pl_capture_metrics.py` 가 "고장이 없을 때 얼마나 잘못 울리는가"(오탐률)를
본다면, 이 도구는 "고장이 있을 때 얼마나 빨리 잡는가"를 본다.

캡처의 `fault_label` 열은 라이브 시나리오가 주입한 고장을 표본마다 기록한다.
라벨이 붙은 연속 구간을 하나의 주입 사건으로 보고, 해당 채널의 보드 신뢰도
상태가 NORMAL 을 벗어나기까지 걸린 표본 수(검출 지연)와, 라벨이 사라진 뒤
NORMAL 로 돌아오기까지 걸린 표본 수(복구 지연)를 센다.

  python fault_latency_metrics.py <pl_capture_*.csv>
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys

from pl_model import CHANNEL_ORDER


STATE_NAME = {0: "NORMAL", 1: "DEGRADED", 2: "INVALID"}
RECOVERY_LIMIT = 200          # 복구를 기다리는 최대 표본 수 (10 초)
SUSTAIN_N = 5                 # 지속 검출 판정 창 (live_scenario_verifier 와 동일)


def load(capture: Path):
    rows = []
    with capture.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            if row.get("fpga_response_valid", "").strip().lower() not in ("1", "true"):
                continue
            try:
                word = int(row["fpga_reliability_word_hex"], 16)
            except (KeyError, ValueError):
                continue
            rows.append({
                "seq": int(row["sample_seq"]),
                "label": (row.get("fault_label") or "").strip(),
                "states": {name: (word >> (i * 2)) & 0x3
                           for i, name in enumerate(CHANNEL_ORDER)},
            })
    return rows


def episodes(rows):
    """라벨이 붙은 연속 구간을 (라벨, 시작 index, 끝 index) 로 묶는다."""
    result = []
    start = None
    for index, row in enumerate(rows):
        label = row["label"]
        if label and start is None:
            start = index
        elif start is not None and label != rows[start]["label"]:
            result.append((rows[start]["label"], start, index - 1))
            start = index if label else None
    if start is not None:
        result.append((rows[start]["label"], start, len(rows) - 1))
    return result


def channel_for(label: str):
    """fault_label 앞부분에서 채널 이름을 찾는다 (예: 'distance:stuck')."""
    for name in CHANNEL_ORDER:
        if label.startswith(name):
            return name
    return None


def analyse(capture: Path):
    rows = load(capture)
    found = episodes(rows)
    results = []

    for label, start, end in found:
        channel = channel_for(label)
        if channel is None:
            results.append({"label": label, "channel": None, "note": "채널 미상"})
            continue

        detect = None
        worst = 0
        for index in range(start, end + 1):
            state = rows[index]["states"][channel]
            worst = max(worst, state)
            if state != 0 and detect is None:
                detect = index - start

        # 주입 진입 순간의 계단(jump/noise)만으로도 상태가 잠깐 흔들린다.
        # 그것을 "검출"로 세면 안 된다.  라이브 검증기와 같은 기준으로
        # 구간 마지막 SUSTAIN_N 표본이 계속 비정상인지를 따로 본다.
        tail = range(max(start, end - SUSTAIN_N + 1), end + 1)
        sustained = all(rows[i]["states"][channel] != 0 for i in tail)

        recover = None
        if detect is not None:
            for index in range(end + 1, min(len(rows), end + 1 + RECOVERY_LIMIT)):
                if rows[index]["states"][channel] == 0:
                    recover = index - end
                    break

        results.append({
            "label": label, "channel": channel,
            "seq_start": rows[start]["seq"], "samples": end - start + 1,
            "detect": detect, "worst": worst, "recover": recover,
            "sustained": sustained,
        })
    return rows, results


def main() -> int:
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except (AttributeError, OSError):
        pass
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    args = parser.parse_args()
    if not args.capture.exists():
        sys.exit(f"캡처 파일이 없다: {args.capture}")

    rows, results = analyse(args.capture)
    print("=" * 84)
    print("고장 주입 검출 / 복구 지연")
    print("=" * 84)
    print(f"캡처 : {args.capture.name}   표본 {len(rows)}")
    if not results:
        print("\nfault_label 이 붙은 구간이 없다. 라이브 시나리오 캡처인지 확인하라.")
        return 0

    print(f"\n{'주입 라벨':28s} {'채널':16s} {'구간':>6s} {'첫검출':>8s} "
          f"{'지속':>6s} {'최악':>9s} {'복구':>8s}")
    missed = []
    transient = []
    detects = []
    recovers = []
    for item in results:
        if item.get("channel") is None:
            continue                      # 시나리오 라벨(노면/시야 등)은 대상이 아니다
        detect = item["detect"]
        recover = item["recover"]
        if detect is None:
            missed.append(item["label"])
            detect_text = "미검출"
        else:
            detects.append(detect)
            detect_text = f"{detect} 표본"
        if detect is not None and not item["sustained"]:
            transient.append(item["label"])
        if recover is None:
            recover_text = "-" if detect is None else "미복구"
        else:
            recovers.append(recover)
            recover_text = f"{recover} 표본"
        print(f"{item['label']:28s} {item['channel']:16s} {item['samples']:6d} "
              f"{detect_text:>8s} {('O' if item['sustained'] else 'X'):>6s} "
              f"{STATE_NAME[item['worst']]:>9s} {recover_text:>8s}")

    channel_events = [i for i in results if i.get("channel")]
    print(f"\n채널 고장 주입 {len(channel_events)}건")
    if detects:
        ordered = sorted(detects)
        print(f"  검출   {len(detects)}건  (중앙 {ordered[len(ordered)//2]} 표본, "
              f"최대 {ordered[-1]} 표본)")
    print(f"  미검출 {len(missed)}건" + (f"  {missed}" if missed else ""))
    if transient:
        print(f"  과도만 {len(transient)}건 (주입 진입 순간에만 떴다가 "
              f"구간 끝에는 NORMAL) {transient}")
    if recovers:
        ordered = sorted(recovers)
        print(f"  복구   {len(recovers)}건  (중앙 {ordered[len(ordered)//2]} 표본, "
              f"최대 {ordered[-1]} 표본)")
    return 1 if (missed or transient) else 0


if __name__ == "__main__":
    sys.exit(main())
