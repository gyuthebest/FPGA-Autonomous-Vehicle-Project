"""같은 주행에 잡음만 바꿔 넣는 통제 실험.

왜 이렇게 해야 하는가
=====================
CARLA 를 배율마다 다시 주행시켜 비교하면 **주행 자체가 달라진다.**
실측(2026-08-14): 배율 OFF/1.0/2.0/3.0 으로 각각 3분씩 주행했더니
accel_x 오탐률이 10.19 / 14.92 / 6.08 / 20.25 % 로 단조롭지 않았다.
운동 채널 오탐은 잡음보다 그날의 선회·가감속에 더 크게 좌우되기 때문이다.
이 상태로 그린 곡선은 강건성이 아니라 주행 차이를 보여줄 뿐이다.

그래서 **무잡음 캡처 하나를 기준 궤적으로 고정**하고, 그 물리값에
계측 잡음만 배율을 바꿔 다시 입혀 REG 워드를 재합성한다.  궤적·날씨·
조향이 모두 동일하므로 차이는 오직 잡음에서 온다.

주의: 기준 캡처의 온도/습도/조도에는 이미 **바닥 진동**(잡음 OFF 에서도
깔리는 성분)이 들어 있다.  그래서 재주입할 때는 계측 성분만 더하고
바닥을 다시 더하지 않는다 (`_offset` 직접 호출).

사용법
------
  python noise_injection_sweep.py <무잡음_캡처.csv> [--scales 0 1 2 3]
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys

from fpga_interface import build_input_words
from compare_golden_vs_pl import decode_input_words
from pl_model import CHANNEL_ORDER, CONSISTENCY_RELATIONS, PLModel
from sensor_noise import SensorNoiseModel, _CHANNELS


WATCH_RELATIONS = (3, 4, 5, 6, 7, 8, 17)


class _Sensor:
    """SensorNoiseModel 이 기대하는 속성 묶음."""
    __slots__ = ("accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z",
                 "temperature", "humidity", "lux", "distance", "approach_speed")


def load_baseline(path: Path):
    rows = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            if row.get("fpga_response_valid", "").strip().lower() not in ("1", "true"):
                continue
            rows.append(row)
    return rows


def evaluate(rows, scale: float, skip: int):
    """배율 scale 로 계측 잡음을 재주입하고 골든 모델을 돌린다."""
    noise = SensorNoiseModel(enabled=True)
    noise.scale = scale
    model = PLModel()
    states = {name: 0 for name in CHANNEL_ORDER}
    invalid = {name: 0 for name in CHANNEL_ORDER}
    confirmed = {number: 0 for number, *_ in CONSISTENCY_RELATIONS}
    counted = 0

    for index, row in enumerate(rows):
        sensor = _Sensor()
        sensor.accel_x = float(row["accel_x_mps2"])
        sensor.accel_y = float(row["accel_y_mps2"])
        sensor.accel_z = float(row["accel_z_mps2"])
        sensor.gyro_x = float(row["gyro_x_rps"])
        sensor.gyro_y = float(row["gyro_y_rps"])
        sensor.gyro_z = float(row["gyro_z_rps"])
        sensor.temperature = float(row["temperature_c"])
        sensor.humidity = float(row["humidity_pct"])
        sensor.lux = float(row["lux"])
        sensor.distance = float(row["distance_m"])
        sensor.approach_speed = float(row["approach_speed_mps"])

        # 계측 성분만 더한다. 바닥 진동은 기준 캡처에 이미 들어 있다.
        if scale:
            for channel in _CHANNELS:
                current = getattr(sensor, channel.attr, None)
                if current is None:
                    continue
                value = float(current) + noise._offset(channel, float(current))
                setattr(sensor, channel.attr,
                        max(channel.low, min(channel.high, value)))
        noise.sample_count += 1
        noise._t += 1.0 / noise.sample_rate_hz

        words = build_input_words(
            sample_seq=index + 1,
            accel_xyz=(sensor.accel_x, sensor.accel_y, sensor.accel_z),
            gyro_xyz=(sensor.gyro_x, sensor.gyro_y, sensor.gyro_z),
            incline_xyz=(float(row["incline_x_deg"]), float(row["incline_y_deg"]),
                         float(row["incline_z_deg"])),
            speed_xyz=(float(row["speed_x_mps"]), float(row["speed_y_mps"]),
                       float(row["speed_z_mps"])),
            distance_m=sensor.distance, approach_speed_mps=sensor.approach_speed,
            temperature=sensor.temperature, humidity_pct=sensor.humidity,
            lux=sensor.lux, weather=int(row["weather"]),
            speed_limit_kmh=float(row["requested_speed_limit_kmh"]),
            rpm_level=int(row["rpm_level"]),
            accelerator=int(row["accelerator_cmd"]), brake=int(row["brake_cmd"]),
            steering_normalized=float(row["steering_normalized"]),
            manual_mode=bool(int(row["manual_mode"])), gear=int(row["gear"]),
            headlight=bool(int(row["headlight"])), hazard=bool(int(row["hazard"])),
            situation=int(row["situation"]),
        )
        sample = decode_input_words(words)
        out = model.step(sample, sample_seq=index + 1,
                         situation=sample["situation"])
        if index < skip:
            continue
        counted += 1
        for name in CHANNEL_ORDER:
            if out.state[name] != 0:
                states[name] += 1
            if out.state[name] == 2:
                invalid[name] += 1
        for number in confirmed:
            if out.cons_err[number]:
                confirmed[number] += 1

    return {"counted": counted, "states": states, "invalid": invalid,
            "confirmed": confirmed}


def main() -> int:
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except (AttributeError, OSError):
        pass
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("baseline", type=Path, help="무잡음으로 뜬 기준 캡처")
    parser.add_argument("--scales", type=float, nargs="+",
                        default=[0.0, 1.0, 2.0, 3.0, 4.0])
    parser.add_argument("--skip", type=int, default=20)
    args = parser.parse_args()
    if not args.baseline.exists():
        sys.exit(f"캡처가 없다: {args.baseline}")

    rows = load_baseline(args.baseline)
    results = [(scale, evaluate(rows, scale, args.skip)) for scale in args.scales]

    width = 9
    header = "".join(f"{('OFF' if s == 0 else f'x{s:g}'):>{width}}"
                     for s, _r in results)
    print("=" * (20 + width * len(results)))
    print("통제 실험 — 같은 주행, 잡음 배율만 변경")
    print("=" * (20 + width * len(results)))
    print(f"기준 궤적 : {args.baseline.name}  "
          f"({results[0][1]['counted']} 표본 비교)")

    print(f"\n[1] 채널별 오탐률")
    print(f"  {'채널':16}{header}")
    for name in CHANNEL_ORDER:
        cells = "".join(
            f"{100.0 * r['states'][name] / max(1, r['counted']):>{width - 1}.2f}%"
            for _s, r in results)
        print(f"  {name:16}{cells}")
    print(f"  {'전체 평균':16}", end="")
    for _s, r in results:
        total = max(1, r["counted"]) * len(CHANNEL_ORDER)
        print(f"{100.0 * sum(r['states'].values()) / total:>{width - 1}.2f}%", end="")
    print()

    print(f"\n[2] INVALID 표본 수")
    print(f"  {'채널':16}{header}")
    any_invalid = False
    for name in CHANNEL_ORDER:
        if not any(r["invalid"][name] for _s, r in results):
            continue
        any_invalid = True
        print(f"  {name:16}" + "".join(f"{r['invalid'][name]:>{width}}"
                                        for _s, r in results))
    if not any_invalid:
        print("  (전 배율에서 0건)")

    relation_channel = {n: c for n, c, *_rest in CONSISTENCY_RELATIONS}
    print(f"\n[3] 관계식별 확정률")
    print(f"  {'관계식':>6} {'채널':14}{header}")
    for number in WATCH_RELATIONS:
        cells = "".join(
            f"{100.0 * r['confirmed'][number] / max(1, r['counted']):>{width - 1}.2f}%"
            for _s, r in results)
        print(f"  {number:>6} {relation_channel[number]:14}{cells}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
