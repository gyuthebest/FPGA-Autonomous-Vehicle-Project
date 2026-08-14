"""추적용 AXI 벡터 생성기 (CARLA 없이 실행).

`pl_verification_logger`가 만드는 벡터 파일과 같은 형식으로,
지정한 주행 시나리오의 REG0..REG9 이미지를 만든다.  실제 캡처를 못 얻는
상황에서도 tb_pl_trace로 PL 판단 과정을 재현할 수 있게 한다.

형식: sample_seq, host_gap_ns, reg0..reg9 (reg는 16진수 8자리)

사용법:
  python make_trace_vectors.py turn    --out vectors_turn.csv
  python make_trace_vectors.py straight --out vectors_straight.csv
  python make_trace_vectors.py brake_ice --out vectors_brake_ice.csv
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys

from fpga_interface import build_input_words
from sensor_noise import SensorNoiseModel

SAMPLE_RATE_HZ = 20.0
DEG_PER_RAD = 180.0 / math.pi

# situation (3비트) — CARLA가 PL로 알려주는 주행 상황.
#   000 정지        (<= 1 km/h)
#   001 장애물 등장  (거리가 200 m sentinel에서 실제 탐지로 전환)
#   010 자세변화    (급격한 각속도 x/y/z 변화)
#   011 날씨 변화
#   100 정상 주행
# PL은 이 값으로 jump/consistency 마스크를 결정한다.  특히
#   consistency_mask_4 = (situation != 000)
# 이 관계식 14(정지 시 gyro_z ~ 0)를 가리므로, 주행 중인데 000을 보내면
# PL은 "정지 중인데 회전한다"고 판정한다.  main.py의 인코딩과 같아야 한다.
SIT_STOPPED, SIT_OBSTACLE, SIT_POSTURE, SIT_WEATHER, SIT_NORMAL = 0, 1, 2, 3, 4
POSTURE_RATE_LIMIT = 0.340          # rad/s, main.py와 동일


def classify_situation(state, prev_distance, index) -> int:
    """main.py의 situation 판정을 그대로 따른다(우선순위 포함)."""
    if (abs(state.gyro_x) > POSTURE_RATE_LIMIT
            or abs(state.gyro_y) > POSTURE_RATE_LIMIT
            or abs(state.gyro_z) > POSTURE_RATE_LIMIT):
        return SIT_POSTURE
    if prev_distance >= 200.0 and state.distance < 200.0:
        return SIT_OBSTACLE
    if abs(state.speed_x) <= 0.278:
        return SIT_STOPPED
    return SIT_NORMAL


class State:
    def __init__(self):
        self.accel_x = self.accel_y = 0.0
        self.accel_z = 9.81
        self.gyro_x = self.gyro_y = self.gyro_z = 0.0
        self.incline_x = self.incline_y = self.incline_z = 0.0
        self.speed_x = self.speed_y = self.speed_z = 0.0
        self.distance = 200.0
        self.approach_speed = 0.0
        self.temperature = 22.0
        self.humidity = 40.0
        self.lux = 30000.0
        self.situation = 0


def scenario_turn(state, index):
    """일정 반경 선회. gyro consistency(관계식 6/7/8)를 자극한다.

    gyro_z = 요각속도[rad/s], incline_z = 요각[deg].
    preprocessor는 pred_gyro_z_1 = (incline_z 차분) * C_GYR(3574)로 기준값을
    만들고, consistency_check는 gyro_z * S_GYR(1024)와 비교한다.
    """
    yaw_rate = 0.50                                   # rad/s (28.6 deg/s)
    state.speed_x = 15.0
    state.gyro_z = yaw_rate
    state.accel_y = yaw_rate * state.speed_x
    state.incline_z = ((index * yaw_rate * DEG_PER_RAD / SAMPLE_RATE_HZ + 180.0)
                       % 360.0) - 180.0


def scenario_straight(state, index):
    state.speed_x = 22.2


def scenario_brake_ice(state, index):
    """저마찰 노면에서 선행차가 접근한다. 제동 중재를 자극한다."""
    state.speed_x = 20.0
    state.temperature = -8.0          # raw -80 -> BLACK ICE 임계
    state.humidity = 95.0
    # 200 m sentinel에서 시작해 접근으로 전환 -> situation 001
    if index < 10:
        state.distance = 200.0
        state.approach_speed = 0.0
    else:
        state.distance = max(8.0, 60.0 - (index - 10) * 1.0)
        state.approach_speed = 20.0


def scenario_standstill_slope(state, index):
    """경사면 정차. 정지 관계식 9~14 와 틸트 관계식 15/16 을 자극한다.

    이 관계식들은 consistency_mask_4/5/6 = (situation != 000) 으로 가려져
    있어 **주행 중에는 한 번도 활성화되지 않는다.**  실주행 캡처
    3,489표본에서 active 표본이 0이었던 이유다.  정차시켜야 열린다.

    가속도는 실제 삼각함수로 넣는다.  PL 의 기준값은 11비트 LUT + 선형보간
    이므로 그 차이가 그대로 잔차로 나타나고, 그것이 TH_ACC_STOP(4 raw =
    0.04 m/s^2) 이 타당한지를 판정하는 근거가 된다.
    """
    incline_x_deg, incline_y_deg = 5.0, 3.0
    state.speed_x = state.speed_y = state.speed_z = 0.0
    state.gyro_x = state.gyro_y = state.gyro_z = 0.0
    state.incline_x, state.incline_y = incline_x_deg, incline_y_deg
    state.incline_z = 0.0
    rad_x, rad_y = math.radians(incline_x_deg), math.radians(incline_y_deg)
    gravity = 9.81
    state.accel_x = -gravity * math.sin(rad_y)
    state.accel_y = gravity * math.sin(rad_x) * math.cos(rad_y)
    state.accel_z = gravity * math.cos(rad_x) * math.cos(rad_y)
    state.distance = 200.0            # 무표적 유지 (situation 001 을 만들지 않는다)
    state.approach_speed = 0.0


SCENARIOS = {
    "turn": scenario_turn,
    "straight": scenario_straight,
    "brake_ice": scenario_brake_ice,
    "standstill_slope": scenario_standstill_slope,
}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenario", choices=sorted(SCENARIOS))
    parser.add_argument("--samples", type=int, default=60)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--no-noise", action="store_true")
    args = parser.parse_args()

    apply_fn = SCENARIOS[args.scenario]
    noise = SensorNoiseModel(enabled=not args.no_noise,
                             sample_rate_hz=SAMPLE_RATE_HZ)
    state = State()

    lines = ["sample_seq,host_gap_ns,reg0,reg1,reg2,reg3,reg4,"
             "reg5,reg6,reg7,reg8,reg9"]
    prev_distance = state.distance
    for index in range(args.samples):
        apply_fn(state, index)
        noise.apply(state)
        # situation은 시나리오가 아니라 상태에서 유도한다. 이전에는 0으로
        # 고정해 두어, 0.5 rad/s로 선회하면서 PL에는 "정지"라고 알리는
        # 모순된 벡터를 만들고 있었다.
        state.situation = classify_situation(state, prev_distance, index)
        prev_distance = state.distance
        words = build_input_words(
            sample_seq=index + 1,
            accel_xyz=(state.accel_x, state.accel_y, state.accel_z),
            gyro_xyz=(state.gyro_x, state.gyro_y, state.gyro_z),
            incline_xyz=(state.incline_x, state.incline_y, state.incline_z),
            speed_xyz=(state.speed_x, state.speed_y, state.speed_z),
            distance_m=state.distance,
            approach_speed_mps=state.approach_speed,
            temperature=state.temperature, humidity_pct=state.humidity,
            lux=state.lux, speed_limit_kmh=50.0, weather=0, rpm_level=2,
            accelerator=8, brake=0, steering_normalized=0.0,
            manual_mode=False, gear=2, headlight=False, hazard=False,
            situation=state.situation,
        )
        gap = 0 if index == 0 else int(1e9 / SAMPLE_RATE_HZ)
        lines.append(f"{index + 1},{gap}," + ",".join(f"{w:08X}" for w in words))

    args.out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"{args.scenario}: {args.samples} samples -> {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
