"""PL 클럭 단위 추적(pl_trace.csv) 분석기.

`sources_1/verification/run_pl_trace.bat`이 만든 CSV를 사람이 읽을 수 있는
형태로 바꾼다.  FPGA 보드가 없어도 PL이 각 표본에서 어떤 근거로 그 결론에
도달했는지 단계별로 확인할 수 있다.

사용법
------
  python analyze_pl_trace.py                     요약 (표본별 판단 결과)
  python analyze_pl_trace.py --changes           신호가 바뀐 클럭만 나열
  python analyze_pl_trace.py --clocks 300 340    지정 구간 원시 덤프
  python analyze_pl_trace.py --gyro              gyro consistency Q-format 검사
  python analyze_pl_trace.py --brake             제동 중재 과정 추적
  python analyze_pl_trace.py --file <경로>       다른 트레이스 파일 사용
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys


DEFAULT_TRACE = (
    Path(__file__).resolve().parent.parent
    / "verification_reports" / "pl_trace.csv"
)

REL_STATE = {0: "NORMAL", 1: "DEGRADED", 2: "INVALID", 3: "INVALID?"}

_RTL = (Path(__file__).resolve().parent.parent
        / "sources_1" / "new" / "sensor_reliability.sv")


def _read_rtl_param(name: str, default: int) -> int:
    """RTL localparam 값을 읽어온다. 하드코딩된 값이 낡는 것을 막는다."""
    try:
        import re
        text = _RTL.read_text(encoding="utf-8", errors="ignore")
        match = re.search(rf"localparam\s+int\s+{name}\s*=\s*(-?\d+)", text)
        return int(match.group(1)) if match else default
    except OSError:
        return default

RISK_TEXT = {
    "s4_collision": ("SAFE", "CAUTION", "DANGER", "CRITICAL", "EMERGENCY"),
    "s4_road_A": ("DRY", "WET", "ICE", "BLACK ICE"),
    "s4_road_B": ("NORMAL", "ROUGH", "SEVERE", "EXTREME"),
    "s4_vision_A": ("BRIGHT", "DIM", "DARK", "VERY DARK"),
    "s4_posture_C": ("SAFE", "CAUTION", "DANGER"),
}

# sensor_reliability.sv의 채널 비트 순서
CHANNELS = ("DIST", "APSP", "AX", "AY", "AZ", "GX", "GY", "GZ",
            "TEMP", "HUM", "LUX")


def load(path: Path):
    if not path.exists():
        sys.exit(
            f"트레이스 파일이 없다: {path}\n"
            "먼저 sources_1/verification/run_pl_trace.bat 을 실행하라."
        )
    def to_int(value):
        # 리셋 구간에서는 신호가 X/Z라 $fwrite("%0d")가 'x'/'z'를 남긴다.
        try:
            return int(value)
        except (TypeError, ValueError):
            return None

    with path.open(newline="", encoding="utf-8") as handle:
        rows = []
        for raw in csv.DictReader(handle):
            row = {k: to_int(v) for k, v in raw.items() if k}
            rows.append({k: v for k, v in row.items() if v is not None})
    if not rows:
        sys.exit("트레이스가 비어 있다.")
    # cycle이 없는 행(완전 X)은 버린다.
    return [row for row in rows if "cycle" in row]


def bitmap(value: int) -> str:
    """채널 비트맵을 이름 목록으로."""
    names = [CHANNELS[i] for i in range(len(CHANNELS)) if value >> i & 1]
    return "+".join(names) if names else "-"


def commits(rows):
    """valid_s1이 올라간 클럭(= 한 표본이 파이프라인에 커밋된 시점)."""
    result = []
    previous = 0
    for row in rows:
        current = row.get("s1_valid_s1", 0)
        if current and not previous:
            result.append(row)
        previous = current
    return result


def cmd_summary(rows) -> None:
    marks = commits(rows)
    print(f"총 {len(rows)} 클럭, 표본 커밋 {len(marks)}회\n")
    header = (f"{'cycle':>7} {'seq':>4} {'dist':>6} {'gyroZ':>6} "
              f"{'range':>6} {'jump':>6} {'stuck':>6} {'noise':>6} {'t/o':>5} "
              f"{'relDIST':>8} {'relGZ':>8} "
              f"{'collision':>10} {'road':>10} {'brake':>5} {'accel':>5} "
              f"{'TD':>2} {'MRM':>3}")
    print(header)
    print("-" * len(header))
    for row in marks:
        print(
            f"{row['cycle']:>7} {row.get('s0_sample_seq_axi', 0):>4} "
            f"{row.get('s1_distance', 0):>6} {row.get('s1_gyro_z', 0):>6} "
            f"{bitmap(row.get('s2_range_err', 0)):>6} "
            f"{bitmap(row.get('s2_jump_err', 0)):>6} "
            f"{bitmap(row.get('s2_stuck_err', 0)):>6} "
            f"{bitmap(row.get('s2_noise_err', 0)):>6} "
            f"{bitmap(row.get('s2_timeout_err', 0)):>5} "
            f"{REL_STATE.get(row.get('s3_rel_distance', 0), '?'):>8} "
            f"{REL_STATE.get(row.get('s3_rel_gyro_z', 0), '?'):>8} "
            f"{RISK_TEXT['s4_collision'][min(row.get('s4_collision', 0), 4)]:>10} "
            f"{RISK_TEXT['s4_road_A'][min(row.get('s4_road_A', 0), 3)]:>10} "
            f"{row.get('s5_final_brake', 0):>5} "
            f"{row.get('s5_final_accel', 0):>5} "
            f"{row.get('s5_td', 0):>2} {row.get('s5_mrm', 0):>3}"
        )


def cmd_changes(rows, watch=None) -> None:
    """값이 바뀐 클럭만 출력한다. '언제 그렇게 판단했는가'를 찾을 때 쓴다."""
    ignore = {"cycle", "sim_ns", "s2_timeout_phase_cnt"}
    columns = [c for c in rows[0] if c not in ignore]
    if watch:
        columns = [c for c in columns if any(w in c for w in watch)]

    previous = None
    count = 0
    for row in rows:
        if previous is None:
            previous = row
            continue
        changed = [
            f"{c}: {previous.get(c)} -> {row.get(c)}"
            for c in columns
            if previous.get(c) != row.get(c)
        ]
        if changed:
            count += 1
            print(f"[clk {row['cycle']:>6} @ {row['sim_ns']:>8} ns] "
                  + ", ".join(changed))
        previous = row
    print(f"\n변화 지점 {count}개")


def cmd_clocks(rows, start, end) -> None:
    selected = [r for r in rows if start <= r["cycle"] <= end]
    if not selected:
        sys.exit(f"{start}~{end} 구간에 클럭이 없다.")
    columns = list(rows[0].keys())
    print(",".join(columns))
    for row in selected:
        print(",".join(str(row.get(c, "")) for c in columns))


def cmd_gyro(rows) -> None:
    """gyro consistency의 좌변/우변을 비교해 Q-format wrap을 검사한다.

    consistency_checker.sv:
        abs(sensor_data * S_GYR - pred_data) <= TH_GYR
    좌변 = gyro_z * 1024, 우변 = pred_gyro_z_1 = delta_incline_z * 3574.
    두 값이 같은 부호로 함께 움직이면 Q-format이 맞는 것이다.
    """
    marks = commits(rows)
    th_gyr = _read_rtl_param("TH_GYR", default=7300)
    print(f"gyro_z consistency 양변 비교 (TH_GYR = {th_gyr})\n")
    header = (f"{'cycle':>7} {'gyro_z':>8} {'좌변(x1024)':>14} "
              f"{'우변(pred)':>14} {'잔차':>12} {'판정':>8}")
    print(header)
    print("-" * len(header))

    wrapped = 0
    for row in marks:
        left = row.get("s1_gyro_z_x_S_GYR", 0)
        right = row.get("s1_pred_gyro_z_1", 0)
        residual = left - right
        verdict = "OK" if abs(residual) <= th_gyr else "초과"
        # 부호가 반대인데 크기가 크면 wrap 의심
        if left and right and (left > 0) != (right > 0) and abs(residual) > th_gyr:
            verdict = "WRAP?"
            wrapped += 1
        print(f"{row['cycle']:>7} {row.get('s1_gyro_z', 0):>8} {left:>14} "
              f"{right:>14} {residual:>12} {verdict:>8}")

    print()
    if wrapped:
        print(f"경고: 부호 반전 의심 {wrapped}건. pred_gyro_*_1 비트폭을 확인하라.")
    else:
        print("부호 반전(wrap) 징후 없음. -> 비트폭은 충분하다.")

    # ---- 임계값 타당성 분석 ----
    # 정착 구간만 본다(첫 표본은 pred가 아직 0이라 잔차가 무의미).
    residuals = [
        abs(r.get("s1_gyro_z_x_S_GYR", 0) - r.get("s1_pred_gyro_z_1", 0))
        for r in marks[1:]
        if r.get("s1_pred_gyro_z_1", 0) != 0
    ]
    preds = sorted({r.get("s1_pred_gyro_z_1", 0) for r in marks[1:]
                    if r.get("s1_pred_gyro_z_1", 0) != 0})
    if not residuals:
        return

    C_GYR = 3574
    steps = sorted({b - a for a, b in zip(preds, preds[1:])})
    print()
    print("임계값 타당성")
    print(f"  잔차 최대 : {max(residuals)}")
    print(f"  잔차 평균 : {sum(residuals) // len(residuals)}")
    print(f"  현재 TH_GYR: {th_gyr}  (sensor_reliability.sv에서 읽음)")
    if steps:
        print(f"  기준값 계단: {steps}  (incline 1 LSB = 0.01 deg -> x C_GYR={C_GYR})")
    print()
    print(f"  기준값(pred)은 incline 1 LSB 단위로만 움직이므로 계단 크기가 {C_GYR}이다.")
    print(f"  따라서 양자화만으로 최소 +-{C_GYR // 2}의 잔차가 항상 생긴다.")
    if th_gyr < C_GYR // 2:
        print(f"  TH_GYR={th_gyr} 은 이 양자화 바닥({C_GYR // 2})보다 작아,")
        print(f"  비트폭을 고쳐도 선회 중 consistency가 구조적으로 통과할 수 없다.")
        print(f"  -> 최소 {C_GYR // 2}, 실측 여유 포함 {max(residuals) + 500} 권장.")
    elif th_gyr > max(residuals):
        print(f"  TH_GYR={th_gyr} 은 양자화 바닥({C_GYR // 2})과 실측 최대"
              f"({max(residuals)})를 모두 넘는다. 적절하다.")
    else:
        print(f"  경고: TH_GYR={th_gyr} 이 실측 최대 잔차({max(residuals)})보다"
              f" 작다. 선회 중 오탐이 남는다.")


def cmd_brake(rows) -> None:
    """제동 중재 과정을 단계별로 보여준다 (마찰 비례 블렌딩 확인용)."""
    marks = commits(rows)
    print("제동 중재: 요청 -> 상한 -> 최종\n")
    header = (f"{'cycle':>7} {'road_A':>10} {'posture_C':>10} "
              f"{'col_brk':>8} {'roadB_brk':>10} {'요청':>6} "
              f"{'노면상한':>9} {'횡상한':>7} {'적용상한':>9} {'최종':>6}")
    print(header)
    print("-" * len(header))
    for row in marks:
        print(
            f"{row['cycle']:>7} "
            f"{RISK_TEXT['s4_road_A'][min(row.get('s5_eff_road_A', 0), 3)]:>10} "
            f"{RISK_TEXT['s4_posture_C'][min(row.get('s5_eff_posture_C', 0), 2)]:>10} "
            f"{row.get('s5_col_brake', 0):>8} {row.get('s5_road_B_brake', 0):>10} "
            f"{row.get('s5_requested_brake', 0):>6} "
            f"{row.get('s5_surface_cap', 0):>9} {row.get('s5_lateral_cap', 0):>7} "
            f"{row.get('s5_brake_cap', 0):>9} {row.get('s5_final_brake', 0):>6}"
        )


def main() -> int:
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except (AttributeError, OSError):
        pass

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--file", type=Path, default=DEFAULT_TRACE)
    parser.add_argument("--changes", action="store_true")
    parser.add_argument("--watch", nargs="*",
                        help="--changes와 함께 쓰면 해당 문자열이 든 신호만")
    parser.add_argument("--clocks", nargs=2, type=int, metavar=("START", "END"))
    parser.add_argument("--gyro", action="store_true")
    parser.add_argument("--brake", action="store_true")
    args = parser.parse_args()

    rows = load(args.file)

    if args.clocks:
        cmd_clocks(rows, args.clocks[0], args.clocks[1])
    elif args.changes:
        cmd_changes(rows, args.watch)
    elif args.gyro:
        cmd_gyro(rows)
    elif args.brake:
        cmd_brake(rows)
    else:
        cmd_summary(rows)
    return 0


if __name__ == "__main__":
    sys.exit(main())
