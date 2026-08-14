"""골든 모델(pl_model.py) 과 PL 출력의 표본 단위 대조.

PL 출력원은 두 가지이며 같은 비교기를 쓴다.

  1) RTL 시뮬레이션 : run_pl_trace.bat 이 만든 pl_trace.csv   (보드 불필요)
  2) 실보드         : fpga_interface 로 받은 응답 로그        (보드 필요)

"FPGA가 우리 로직을 그대로 반영했는가" 를 판정하는 도구다.
불일치가 나오면 원인은 셋 중 하나이고, 어느 쪽이든 반드시 규명해야 한다.

  - RTL이 의도한 알고리즘과 다르게 구현됨
  - 합성/타이밍/AXI 전송 문제
  - 골든 모델이 사양을 잘못 옮김

파이프라인 정렬
--------------
RTL의 S2~S5 단계는 valid에 맞춰 레지스터링되므로, 커밋 클럭 그 순간에는 아직
이전 표본의 값이 남아 있다.  그래서 표본 N의 RTL 값은 **다음 커밋 직전 행**
에서 읽는다(정착값).

사용법
------
  python compare_golden_vs_pl.py --vectors <벡터.csv> [--trace <pl_trace.csv>]
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys

from pl_model import (
    CHANNEL_ORDER, MODEL_COVERAGE, PLModel, blend_brake,
)


ROOT = Path(__file__).resolve().parent.parent
DEFAULT_TRACE = ROOT / "verification_reports" / "pl_trace.csv"

REL_NAME = {0: "NORMAL", 1: "DEGRADED", 2: "INVALID"}


def _signed(value: int, bits: int) -> int:
    value &= (1 << bits) - 1
    return value - (1 << bits) if value & (1 << (bits - 1)) else value


def decode_input_words(words) -> dict:
    """REG0..REG9 -> PL이 보는 정수 채널값 (build_input_words 의 역변환)."""
    reg0, reg1, reg2, reg3, reg4, reg5, reg6, reg7, reg8, _reg9 = words
    return {
        "accel_x": _signed(reg0 & 0xFFFF, 16),
        "accel_y": _signed((reg0 >> 16) & 0xFFFF, 16),
        "accel_z": _signed(reg1 & 0xFFFF, 16),
        "gyro_x": _signed((reg1 >> 16) & 0xFFFF, 16),
        "gyro_y": _signed(reg2 & 0xFFFF, 16),
        "gyro_z": _signed((reg2 >> 16) & 0xFFFF, 16),
        "incline_x": _signed(reg3 & 0xFFFF, 16),
        "incline_y": _signed((reg3 >> 16) & 0xFFFF, 16),
        "incline_z": _signed(reg4 & 0xFFFF, 16),
        # 속도는 상위 8비트(reg4/reg6)와 하위 6비트(reg8)로 나뉘어 온다.
        #   speed_x = $signed({slv_reg4[23:16], slv_reg8[11:6]})
        "speed_x": _signed(((reg4 >> 16) & 0xFF) << 6 | ((reg8 >> 6) & 0x3F), 14),
        "speed_y": _signed(((reg4 >> 24) & 0xFF) << 6 | ((reg8 >> 12) & 0x3F), 14),
        "distance": reg5 & 0x7FFF,
        # 압축 필드는 AXI 언패킹에서 다시 왼쪽으로 시프트된다.
        #   approach_speed = $signed({slv_reg5[24:15], 3'b0})
        # 시프트를 빠뜨리면 PL이 보는 값보다 8배 작아진다.
        "approach_speed": _signed((reg5 >> 15) & 0x3FF, 10) << 3,
        "humidity": (reg5 >> 25) & 0x7F,
        "lux": reg6 & 0x3FFFF,
        "speed_z": _signed(((reg6 >> 18) & 0xFF) << 6 | ((reg8 >> 18) & 0x3F), 14),
        "weather": (reg6 >> 30) & 0x3,
        "temperature": _signed(reg7 & 0x7FF, 11),
        #   steering = $signed({slv_reg7[23:19], slv_reg8[26:24]})
        "steering": _signed(((reg7 >> 19) & 0x1F) << 3 | ((reg8 >> 24) & 0x7), 8),
        "manual_mode": reg8 & 1,
        "situation": (reg8 >> 3) & 0x7,
    }


# each_sensor_check 는 표본이 끊긴 동안 100 ms 마다 timeout 증거를 한 번 쌓는다
# (UPDATE_CLK_X2 = 2 * 표본주기).  호스트 지터로 이 100 ms 를 넘기는 프레임이
# 실제로 발생하고, 그 때 tm1/tm2 가 뜨면서 다음 1~2 표본의 jump/stuck/noise/
# consistency 증거가 보류된다.  즉 디바운스 위상이 어긋난다.
#
# 그런데 캡처가 남기는 것은 **호스트 송신 시각**이고, PL 이 실제로 침묵을
# 관측한 구간은 PS 수신/AXI 기록 시각 기준이다.  100~120 ms 처럼 경계에 걸친
# 간격은 보드가 틱을 냈는지가 sub-ms 정렬에 좌우되어 로그만으로 복원되지
# 않는다 (실측: 임계 100 ms 로 복원하면 불일치 43, 120 ms 면 34, 아예 무시하면
# 32).  그래서 기본값은 "추정하지 않는다" 이고, 대신 불일치가 지터 사건과
# 겹치는지를 보고한다.  --jitter-ticks 로 복원을 켤 수 있다.
TIMEOUT_PHASE_MS = 100.0


def timeout_ticks_for_gap(row) -> int:
    raw = (row.get("host_gap_ms") or "").strip()
    if not raw:
        return 0
    try:
        gap_ms = float(raw)
    except ValueError:
        return 0
    return int(gap_ms // TIMEOUT_PHASE_MS) if gap_ms > 0 else 0


def load_vectors(path: Path):
    rows = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            words = [int(row[f"reg{i}"], 16) for i in range(10)]
            rows.append((int(row["sample_seq"]), words))
    return rows


def load_trace(path: Path):
    def to_int(value):
        try:
            return int(value)
        except (TypeError, ValueError):
            return None

    with path.open(newline="", encoding="utf-8") as handle:
        rows = []
        for raw in csv.DictReader(handle):
            row = {k: to_int(v) for k, v in raw.items() if k}
            row = {k: v for k, v in row.items() if v is not None}
            if "cycle" in row:
                rows.append(row)
    return rows


def settled_rows(trace):
    """표본별 정착값: 각 커밋 구간의 마지막 행."""
    commits = []
    previous = 0
    for index, row in enumerate(trace):
        current = row.get("s1_valid_s1", 0)
        if current and not previous:
            commits.append(index)
        previous = current

    result = []
    for order, start in enumerate(commits):
        end = commits[order + 1] - 1 if order + 1 < len(commits) else len(trace) - 1
        result.append(trace[end])
    return result


# 골든 모델 값 -> 트레이스 컬럼
def build_expectations(out, sample):
    return [
        ("delta_distance", out.delta["distance"], "s1_delta_distance"),
        ("delta_gyro_z", out.delta["gyro_z"], "s1_delta_gyro_z"),
        ("delta_accel_z", out.delta["accel_z"], "s1_delta_accel_z"),
        ("delta_temperature", out.delta["temperature"], "s1_delta_temp"),
        ("pred_gyro_z", out.pred["gyro_z"], "s1_pred_gyro_z_1"),
        ("pred_distance", out.pred["distance"], "s1_pred_distance"),
        ("range_err", out.range_err, "s2_range_err"),
        ("jump_err", out.jump_err, "s2_jump_err"),
        ("stuck_err", out.stuck_err, "s2_stuck_err"),
        ("noise_err", out.noise_err, "s2_noise_err"),
        ("timeout_err", out.timeout_err, "s2_timeout_err"),
        # cons_err_gyro_z = {관계식17, 관계식14, 관계식8} 3비트 전부 비교한다.
        ("cons_gyro_z", out.cons_gyro_z, "s2b_cons_gyro_z"),
        # cons_err_accel_x = {관계식16(틸트), 관계식9(정지), 관계식3(동역학)}
        # cons_err_distance = 관계식1.  둘 다 지금까지 비교 대상이 아니었다.
        ("cons_accel_x", (int(out.cons_err[3])
                          | (int(out.cons_err[9]) << 1)
                          | (int(out.cons_err[16]) << 2)), "s2b_cons_accel_x"),
        ("cons_distance", int(out.cons_err[1]), "s2b_cons_distance"),
        ("rel_distance", out.state["distance"], "s3_rel_distance"),
        ("rel_gyro_z", out.state["gyro_z"], "s3_rel_gyro_z"),
        ("rel_accel_x", out.state["accel_x"], "s3_rel_accel_x"),
        ("rel_accel_y", out.state["accel_y"], "s3_rel_accel_y"),
        ("rel_accel_z", out.state["accel_z"], "s3_rel_accel_z"),
        ("rel_temperature", out.state["temperature"], "s3_rel_temp"),
        ("rel_humidity", out.state["humidity"], "s3_rel_hum"),
        ("risk_collision", out.risk.collision, "s4_collision"),
        ("risk_road_A", out.risk.road_A, "s4_road_A"),
        ("risk_road_B", out.risk.road_B, "s4_road_B"),
        ("risk_vision_A", out.risk.vision_A, "s4_vision_A"),
        ("risk_posture_C", out.risk.posture_C, "s4_posture_C"),
    ]


# ---------------------------------------------------------------------------
# 실보드 모드
# ---------------------------------------------------------------------------

def compare_board(capture: Path, skip: int = 20, jitter_ticks: bool = False) -> int:
    """실보드 캡처(pl_verification_logger 출력)와 골든 모델을 대조한다.

    보드가 AXI로 노출하는 신뢰도 워드(read_reg10)는 채널별 2비트 상태다.
    sensor_input_v1_0_S00_AXI.v:
        {10'b0, lux, humidity, temperature, gyro_z, gyro_y, gyro_x,
         accel_z, accel_y, accel_x, approach_speed, distance}
    이 순서는 pl_model.CHANNEL_ORDER 와 정확히 일치한다.
    """
    model = PLModel()
    total = 0
    skipped = 0
    warmed = 0
    mismatches = []
    per_channel = {}
    bad_seqs = []
    jitter_seqs = []
    max_gap_ms = 0.0
    # 신뢰도 워드와 별개로 대조하는 항목들.
    #   risk 워드 / HUD 는 조합 논리라 비트 단위로 맞아야 한다.
    #   TD/MRM 은 PL 자유 카운터 위상에 걸려 ±1틱 오차가 남는다.
    extra_total = {"risk_word": 0, "hud": 0, "td": 0, "mrm": 0}
    extra_bad = {"risk_word": 0, "hud": 0, "td": 0, "mrm": 0}
    td_off_by_one = 0
    prev_send_ns = None

    with capture.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            if row.get("fpga_response_valid", "").strip().lower() not in ("1", "true"):
                skipped += 1
                continue
            words = [int(row[f"reg{i}_hex"], 16) for i in range(10)]
            sample = decode_input_words(words)
            seq_value = int(row["sample_seq"])
            ticks = timeout_ticks_for_gap(row)
            try:
                max_gap_ms = max(max_gap_ms, float(row.get("host_gap_ms") or 0))
            except ValueError:
                pass
            if ticks:
                jitter_seqs.append(seq_value)
                if jitter_ticks:
                    for _ in range(ticks):
                        model.tick_missing_sample()

            # TD/MRM 1초 타이머는 실시간 기준이므로 캡처의 송신 시각으로 돌린다.
            try:
                send_ns = int(row["host_send_ns"])
            except (KeyError, ValueError):
                send_ns = None
            if send_ns is not None:
                if prev_send_ns is not None:
                    model.advance_time(send_ns - prev_send_ns)
                prev_send_ns = send_ns

            out = model.step(sample, sample_seq=seq_value,
                             situation=sample.get("situation", 0))

            # 예열 구간은 모델을 진행시키되 비교하지 않는다.  보드는 시험 전에
            # 표본을 받지 못해 transport timeout이 확정된 상태(전 채널 INVALID)
            # 이고 모델에는 그 사전 상태가 없다.  timeout 치유(5표본)와
            # noise 창(10표본)이 지나야 양쪽이 같은 조건에서 출발한다.
            if warmed < skip:
                warmed += 1
                continue

            def _flag(column):
                value = (row.get(column) or "").strip().lower()
                return value in ("1", "true")

            if row.get("fpga_risk_word_hex"):
                extra_total["risk_word"] += 1
                if (int(row["fpga_risk_word_hex"], 16) & 0xFFFF) != out.risk_word:
                    extra_bad["risk_word"] += 1
            if row.get("fpga_hud_warning") is not None:
                extra_total["hud"] += 1
                if _flag("fpga_hud_warning") != out.hud_warning:
                    extra_bad["hud"] += 1
            if row.get("fpga_mrm") is not None:
                extra_total["mrm"] += 1
                if _flag("fpga_mrm") != out.mrm:
                    extra_bad["mrm"] += 1
            board_td = (row.get("fpga_td_remain_sec") or "").strip()
            if board_td.isdigit():
                extra_total["td"] += 1
                delta = abs(int(board_td) - out.td_remain_sec)
                if delta:
                    extra_bad["td"] += 1
                    if delta == 1:
                        td_off_by_one += 1

            board_word = int(row["fpga_reliability_word_hex"], 16)
            for index, name in enumerate(CHANNEL_ORDER):
                board_state = (board_word >> (index * 2)) & 0x3
                expected = out.state[name]
                total += 1
                if board_state != expected:
                    per_channel[name] = per_channel.get(name, 0) + 1
                    bad_seqs.append(seq_value)
                    if len(mismatches) < 20:
                        mismatches.append(
                            f"seq{seq_value} {name}: "
                            f"모델={REL_NAME.get(expected)} "
                            f"보드={REL_NAME.get(board_state)}"
                        )

    print("=" * 74)
    print("골든 모델 vs 실보드 신뢰도 워드 대조")
    print("=" * 74)
    print(f"캡처     : {capture.name}")
    print(f"예열 제외 : {warmed} 표본")
    print(f"비교 항목 : {total}  (응답 없음으로 건너뜀: {skipped})")
    print(f"전송 지터 : 최대 {max_gap_ms:.1f} ms, "
          f"{TIMEOUT_PHASE_MS:.0f} ms 초과 {len(jitter_seqs)}건"
          + ("  (틱 복원 켬)" if jitter_ticks else ""))

    if any(extra_total.values()):
        print("\n신뢰도 워드 외 대조")
        label = {"risk_word": "risk 워드(유효 tier)", "hud": "HUD 경고",
                 "mrm": "MRM", "td": "TD 잔여초"}
        for key in ("risk_word", "hud", "mrm", "td"):
            if not extra_total[key]:
                continue
            bad, total_key = extra_bad[key], extra_total[key]
            note = ""
            if key == "td" and bad:
                note = (f"  (그중 ±1초 {td_off_by_one}건 — PL 자유 카운터 "
                        f"위상은 복원 불가)")
            verdict = "일치" if bad == 0 else f"불일치 {bad}"
            print(f"  {label[key]:22s} {total_key:6d}건 중 {verdict}"
                  f"  ({100.0 * (total_key - bad) / total_key:.2f}%){note}")

    if not mismatches:
        print("\n결과: PASS - 보드 신뢰도 판정이 골든 모델과 완전히 일치한다.")
        return 0

    total_bad = sum(per_channel.values())
    rate = 100.0 * total_bad / total if total else 0.0
    print(f"\n결과: 불일치 {total_bad}건 / {total} ({rate:.3f}%)")
    print(f"  발생 구간: seq {min(bad_seqs)} ~ {max(bad_seqs)}")
    print("  채널별:")
    for name, count in sorted(per_channel.items(), key=lambda kv: -kv[1]):
        print(f"    {name:16s} {count}")
    print("  예시 (최대 20건):")
    for item in mismatches:
        print(f"    - {item}")

    # 불일치가 전부 지터 사건 근처면 원인은 RTL 이 아니라 전송이다.
    if jitter_seqs:
        unique_bad = sorted(set(bad_seqs))
        distances = [min(abs(seq - j) for j in jitter_seqs) for seq in unique_bad]
        near = sum(1 for d in distances if d <= 10)
        print(f"\n  지터 상관: 불일치 표본 {len(unique_bad)}개 중 {near}개가 "
              f"{TIMEOUT_PHASE_MS:.0f} ms 초과 사건 10표본 이내"
              f" (최대 거리 {max(distances)})")
        if near == len(unique_bad):
            print("  -> 전 불일치가 전송 지터 구간에 몰려 있다. "
                  "지터 없는 캡처로 재판정하라 (board_smoke_test.py).")
    return 1


def main() -> int:
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except (AttributeError, OSError):
        pass

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vectors", type=Path)
    parser.add_argument("--board", type=Path,
                        help="실보드 캡처 CSV (pl_verification_logger 출력)")
    parser.add_argument("--skip", type=int, default=20,
                        help="비교에서 제외할 예열 표본 수 (기본 20)")
    parser.add_argument("--jitter-ticks", action="store_true",
                        help="호스트 송신 간격에서 timeout 틱을 추정해 모델에 주입한다 "
                             "(경계 구간은 복원되지 않으므로 기본은 끔)")
    parser.add_argument("--trace", type=Path, default=DEFAULT_TRACE)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    if args.board:
        if not args.board.exists():
            sys.exit(f"캡처 파일이 없다: {args.board}")
        return compare_board(args.board, skip=args.skip,
                             jitter_ticks=args.jitter_ticks)

    if not args.vectors:
        sys.exit("--vectors 또는 --board 중 하나가 필요하다.")
    if not args.vectors.exists():
        sys.exit(f"벡터 파일이 없다: {args.vectors}")
    if not args.trace.exists():
        sys.exit(f"트레이스가 없다: {args.trace}\nrun_pl_trace.bat 을 먼저 실행하라.")

    vectors = load_vectors(args.vectors)
    settled = settled_rows(load_trace(args.trace))

    print("=" * 74)
    print("골든 모델 vs PL(RTL 시뮬레이션) 표본 단위 대조")
    print("=" * 74)
    print(f"벡터   : {args.vectors.name}  ({len(vectors)} 표본)")
    print(f"트레이스: {args.trace.name}  (정착 표본 {len(settled)}개)")

    if len(settled) != len(vectors):
        print(f"\n주의: 표본 수 불일치 (벡터 {len(vectors)}, 트레이스 {len(settled)}). "
              f"겹치는 구간만 비교한다.")

    model = PLModel()
    count = min(len(vectors), len(settled))
    mismatches = {}
    compared = 0

    for index in range(count):
        seq, words = vectors[index]
        sample = decode_input_words(words)
        out = model.step(sample, sample_seq=seq,
                         situation=sample.get("situation", 0))
        row = settled[index]

        for item in build_expectations(out, sample):
            label, expected, column = item[0], item[1], item[2]
            mask = item[3] if len(item) > 3 else None
            if column not in row:
                continue
            compared += 1
            actual = row[column] if mask is None else (row[column] & mask)
            if actual != expected:
                record = mismatches.setdefault(label, [])
                if len(record) < 3:
                    record.append((seq, expected, actual))
                elif len(record) == 3:
                    record.append(None)     # 생략 표시

    print(f"\n비교한 항목 수: {compared}")
    if not mismatches:
        print("\n결과: PASS — 모든 비교 항목이 골든 모델과 일치한다.")
    else:
        print(f"\n결과: 불일치 {len(mismatches)}종")
        for label, samples in sorted(mismatches.items()):
            shown = [s for s in samples if s]
            detail = ", ".join(
                f"seq{seq}: 모델={exp} PL={act}" for seq, exp, act in shown
            )
            more = " ..." if len(samples) > len(shown) else ""
            print(f"  - {label:18s} {detail}{more}")

    print("\n모델 커버리지")
    for item, level in MODEL_COVERAGE.items():
        print(f"  {item:42s} {level}")

    return 1 if mismatches else 0


if __name__ == "__main__":
    sys.exit(main())
