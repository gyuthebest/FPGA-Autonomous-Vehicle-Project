"""잡음 배율별 캡처를 나란히 놓고 강건성 곡선을 만든다.

배경
====
"우리 시스템이 센서 잡음에 강건한가"는 오탐률 하나로는 답이 안 된다.
잡음을 키우면 오탐이 느는 것은 당연하고, 진짜 질문은 두 가지다.

  1) 오탐률이 **어느 배율에서 급증하는가** (무릎)
  2) 그 사이 **검출 능력이 유지되는가**

임계값을 올리면 1번은 좋아지지만 2번이 나빠진다.  둘을 같이 봐야
"강건하다"는 말에 의미가 생긴다.

이 도구는 1번을 담당한다.  2번은 `fault_latency_metrics.py` 가
고장 주입 캡처에서 산출한다.

사용법
------
  python noise_robustness_report.py \
      off=logs/.../pl_capture_A.csv 1.0=...B.csv 2.0=...C.csv

라벨은 아무 문자열이나 되고, 순서대로 표에 나온다.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

from pl_model import CHANNEL_ORDER, CONSISTENCY_RELATIONS
from pl_capture_metrics import analyse


# 정상 주행에서 잔차가 임계를 넘기 시작하면 곧바로 오탐으로 이어지는
# 관계식들.  전체 17개를 다 찍으면 표가 읽히지 않는다.
WATCH_RELATIONS = (3, 4, 5, 6, 7, 8, 17)


def main() -> int:
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except (AttributeError, OSError):
        pass

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("captures", nargs="+", metavar="라벨=경로")
    parser.add_argument("--skip", type=int, default=20)
    args = parser.parse_args()

    columns = []
    for item in args.captures:
        if "=" not in item:
            sys.exit(f"'라벨=경로' 형식이어야 한다: {item}")
        label, _, raw_path = item.partition("=")
        path = Path(raw_path)
        if not path.exists():
            sys.exit(f"캡처가 없다: {path}")
        columns.append((label, analyse(path, skip=args.skip)))

    width = max(9, max(len(label) for label, _ in columns) + 2)

    print("=" * (26 + width * len(columns)))
    print("잡음 배율별 강건성")
    print("=" * (26 + width * len(columns)))
    header = "".join(f"{label:>{width}}" for label, _ in columns)
    for label, result in columns:
        print(f"  {label:>8} : {result['capture']}  "
              f"({result['samples_counted']} 표본)")

    print(f"\n[1] 채널별 오탐률 (고장 미주입이므로 NORMAL 이외는 전부 오탐)")
    print(f"  {'채널':16}{header}")
    for name in CHANNEL_ORDER:
        cells = ""
        for _label, result in columns:
            counts = result["states"][name]
            total = max(1, result["samples_counted"])
            bad = counts[1] + counts[2]
            cells += f"{100.0 * bad / total:>{width - 1}.2f}%"
        print(f"  {name:16}{cells}")

    print(f"\n  {'전체 평균':16}", end="")
    for _label, result in columns:
        total = max(1, result["samples_counted"]) * len(CHANNEL_ORDER)
        bad = sum(c[1] + c[2] for c in result["states"].values())
        print(f"{100.0 * bad / total:>{width - 1}.2f}%", end="")
    print()

    print(f"\n[2] INVALID 표본 (안전 판정을 멈추는 등급)")
    print(f"  {'채널':16}{header}")
    for name in CHANNEL_ORDER:
        if not any(r["states"][name][2] for _l, r in columns):
            continue
        cells = "".join(f"{r['states'][name][2]:>{width}}" for _l, r in columns)
        print(f"  {name:16}{cells}")
    if not any(r["states"][n][2] for _l, r in columns for n in CHANNEL_ORDER):
        print("  (전 배율에서 INVALID 0건)")

    relation_channel = {num: ch for num, ch, *_rest in CONSISTENCY_RELATIONS}
    print(f"\n[3] 관계식별 확정률")
    print(f"  {'관계식':>6} {'채널':14}{header}")
    for num in WATCH_RELATIONS:
        cells = ""
        for _label, result in columns:
            item = result["relations"].get(num) or result["relations"].get(str(num))
            total = max(1, result["samples_counted"])
            cells += f"{100.0 * item['confirmed'] / total:>{width - 1}.2f}%"
        print(f"  {num:>6} {relation_channel[num]:14}{cells}")

    print(f"\n[4] 관계식별 잔차 p90 / 임계값")
    print(f"  {'관계식':>6} {'임계':>7}{header}")
    for num in WATCH_RELATIONS:
        first = columns[0][1]["relations"].get(num) or \
            columns[0][1]["relations"].get(str(num))
        cells = ""
        for _label, result in columns:
            item = result["relations"].get(num) or result["relations"].get(str(num))
            value = item["residual_p90"]
            cells += f"{'-' if value is None else value:>{width}}"
        print(f"  {num:>6} {first['threshold']:>7}{cells}")

    print("\n해석")
    print("  [1] 오탐률이 급증하는 배율이 강건성 한계다.")
    print("  [3][4] 잔차 p90 이 임계값을 넘어서는 순간 확정률이 따라 오른다.")
    print("  검출 능력 유지 여부는 fault_latency_metrics.py 로 따로 확인하라.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
