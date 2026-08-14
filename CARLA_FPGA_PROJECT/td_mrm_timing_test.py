"""TD 카운트다운과 MRM 발동 시점을 실보드에서 측정한다 (CARLA 불필요).

확인하려는 것
-------------
risk_control.sv 는 다음과 같이 되어 있다.

    td_condition       = (Re_collision|Re_posture_A|B|C 중 하나라도 INVALID)
    transition_demand  = (td_remain_sec <= 10)
    mrm                = (td_remain_sec == 0)

`td_remain_sec` 는 11 에서 시작해 1초에 1씩 줄어든다.  따라서 INVALID 가
확정된 뒤 **약 10초 후에 MRM** 이 되어야 하고, INVALID 즉시 MRM 이 되면
안 된다.  이 스크립트는 그 시간 간격을 실측한다.

자극
----
gyro_x 를 한 값에 고정하고 incline_x(기준값)만 계속 움직인다.  실제 stuck
센서와 같은 조건이며, stuck 확정 후 관계식 6(동역학)까지 깨져 고장 2개가
되어 gyro_x 가 INVALID -> Re_posture_A INVALID -> td_condition 성립.

MRM 중 기어도 함께 기록한다.  risk_control.sv 는 MRM 일 때
`rpm <= 1 && can_downshift && gear > 0` 이면 한 단 낮추는데, 이 조건이
실제로 성립하는지 본다.
"""

from __future__ import annotations

import argparse
import sys
import time

from fpga_interface import FPGAInterface, build_input_words


def frame(seq, gyro_x, incline_x, rpm_level, gear):
    return build_input_words(
        sample_seq=seq,
        accel_xyz=(0.0, 0.0, 9.81), gyro_xyz=(gyro_x, 0.0, 0.0),
        incline_xyz=(incline_x, 0.0, 0.0), speed_xyz=(20.0, 0.0, 0.0),
        distance_m=200.0, approach_speed_mps=0.0,
        temperature=22.0, humidity_pct=40.0, lux=30000.0,
        speed_limit_kmh=50.0, weather=0, rpm_level=rpm_level,
        accelerator=8, brake=0, steering_normalized=0.0,
        manual_mode=False, gear=gear, headlight=False, hazard=False,
        situation=4,
    )


def main() -> int:
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except (AttributeError, OSError):
        pass

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ip", default="192.168.1.10")
    parser.add_argument("--port", type=int, default=5001)
    parser.add_argument("--local-port", type=int, default=5002)
    parser.add_argument("--warmup", type=int, default=25)
    parser.add_argument("--seconds", type=float, default=20.0,
                        help="자극 유지 시간 (기본 20초 = TD 10초 + 여유)")
    parser.add_argument("--rpm", type=int, default=2,
                        help="rpm_level 0..3 (MRM 다운시프트 조건은 rpm<=1)")
    parser.add_argument("--gear", type=int, default=3,
                        help="PL 로 보내는 기어 0..3")
    args = parser.parse_args()

    fpga = FPGAInterface(board_ip=args.ip, board_port=args.port,
                         local_port=args.local_port, timeout_s=0.2, enabled=True)

    print("=" * 78)
    print("TD 카운트다운 / MRM 발동 시점 실측")
    print("=" * 78)
    print(f"자극: gyro_x 고정 + incline_x 변화 (stuck + 관계식6 -> gyro_x INVALID)")
    print(f"rpm_level={args.rpm}  gear={args.gear}  (MRM 다운시프트 조건 rpm<=1)")

    seq = 0
    try:
        # 1) 정상 표본으로 예열
        for _ in range(args.warmup):
            seq += 1
            fpga.exchange(frame(seq, 0.0, 0.0, args.rpm, args.gear), seq)
            time.sleep(0.05)

        print(f"\n{'경과(s)':>7} {'seq':>5} {'gyroX신뢰도':>10} {'TD':>3} "
              f"{'잔여초':>5} {'MRM':>4} {'accel':>6} {'brake':>6} {'gear':>5}")
        print("-" * 78)

        start = time.perf_counter()
        invalid_at = None
        td_at = None
        mrm_at = None
        previous_row = None
        samples = int(args.seconds / 0.05)

        for index in range(samples):
            seq += 1
            incline = (index * 0.5) % 20.0          # 기준값은 계속 변한다
            result = fpga.exchange(
                frame(seq, 0.30, incline, args.rpm, args.gear), seq)
            elapsed = time.perf_counter() - start
            if result is None:
                time.sleep(0.05)
                continue

            gyro_x_state = (result.reliability_word >> (5 * 2)) & 0x3
            row = (gyro_x_state, result.transition_demand,
                   result.td_remain_sec, result.mrm,
                   result.accelerator, result.brake, result.gear)

            if gyro_x_state == 2 and invalid_at is None:
                invalid_at = elapsed
            if result.transition_demand and td_at is None:
                td_at = elapsed
            if result.mrm and mrm_at is None:
                mrm_at = elapsed

            if row != previous_row:
                name = {0: "NORMAL", 1: "DEGRADED", 2: "INVALID"}.get(gyro_x_state, "?")
                print(f"{elapsed:>7.2f} {seq:>5} {name:>10} "
                      f"{int(result.transition_demand):>3} {result.td_remain_sec:>5} "
                      f"{int(result.mrm):>4} {result.accelerator:>6} "
                      f"{result.brake:>6} {result.gear:>5}")
                previous_row = row
            time.sleep(0.05)
    finally:
        fpga.close()

    print("\n" + "=" * 78)
    print("판정")
    print("=" * 78)
    if invalid_at is None:
        print("  자극이 gyro_x INVALID 를 만들지 못했다. 시간을 늘려라.")
        return 1
    print(f"  gyro_x INVALID 확정   : {invalid_at:.2f} s")
    print(f"  TD(전환요구) 시작     : {td_at:.2f} s" if td_at else "  TD 미발동")
    if mrm_at is None:
        print(f"  MRM 발동              : 미발동 (관측 {args.seconds:.0f}초 내)")
    else:
        print(f"  MRM 발동              : {mrm_at:.2f} s")
        print(f"  INVALID -> MRM 간격   : {mrm_at - invalid_at:.2f} s")
        if mrm_at - invalid_at < 5.0:
            print("\n  => 문제: INVALID 직후 MRM 이 발동한다. 10초 카운트다운이 "
                  "동작하지 않는다.")
        else:
            print("\n  => 정상: 10초 카운트다운을 거쳐 MRM 이 발동한다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
