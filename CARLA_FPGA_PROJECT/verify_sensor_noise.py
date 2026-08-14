"""보드 없이 실행하는 센서 잡음 / AXI 패킹 검증.

이 스크립트는 `sensor_checker.sv`의 range / jump / stuck / noise 판정을
Python으로 그대로 재현하고, `fpga_interface.build_input_words`가 실제로
만들어내는 AXI 레지스터 이미지에서 채널값을 디코드해 검사한다.  따라서
CARLA도 FPGA 보드도 Vivado도 필요 없다.

재현한 RTL (sources_1/new/sensor_checker.sv)
--------------------------------------------
    range_error = (range_cnt >= RANGE_N)
    raw_range   = (USE_MIN && data < MIN) || (USE_MAX && data > MAX)

    jump_error  = (jump_cnt >= JUMP_N)
    raw_jump    = |processed - prev_processed| > JUMP_THRESHOLD   (2차 차분)

    stuck_error = (stuck_cnt >= STUCK_N)
    raw_stuck   = 01(누적) if processed == 0 and testable
                  00(감소) if processed != 0
    testable    = (CHANNEL_TYPE_2 == 0) or |trig_val| >= STUCK_THRESHOLD

    noise_error = (sum(|delta|,10) > NOISE_THRESHOLD_1 * 10)
              and (sign_flips(10) > NOISE_THRESHOLD_2)

카운터는 모두 codex가 추가한 포화 동작을 포함한다.

실행:  python verify_sensor_noise.py
"""

from __future__ import annotations

import sys

from fpga_interface import build_input_words
from sensor_noise import SensorNoiseModel


SAMPLE_RATE_HZ = 20.0
SAMPLES = 400            # 20초
HISTORY = 10

# sensor_reliability.sv 공통 디바운스 상수
RU, RD, RN = 1, 1, 3
JU, JD, JN = 6, 1, 18


class Channel:
    """each_sensor_check 한 인스턴스의 파라미터."""

    def __init__(self, name, range_min, range_max, use_min, use_max,
                 jump_threshold, stuck_threshold, noise_1, noise_2,
                 stuck_u, stuck_d, stuck_n, channel_type_2, trig=None):
        self.name = name
        self.range_min = range_min
        self.range_max = range_max
        self.use_min = use_min
        self.use_max = use_max
        self.jump_threshold = jump_threshold
        self.stuck_threshold = stuck_threshold
        self.noise_1 = noise_1
        self.noise_2 = noise_2
        self.stuck_u = stuck_u
        self.stuck_d = stuck_d
        self.stuck_n = stuck_n
        self.channel_type_2 = channel_type_2
        self.trig = trig          # stuck testable 판정에 쓰는 신호 이름


# sensor_reliability.sv의 each_sensor_check 인스턴스와 1:1 대응
CHANNELS = (
    Channel("accel_x", -1600, 1600, True, True, 2000, 2, 500, 7, 1, 1, 10, 1, "speed_x"),
    Channel("accel_y", -1600, 1600, True, True, 2000, 2, 500, 7, 1, 1, 10, 1, "speed_x"),
    Channel("accel_z", -1600, 1600, True, True, 2000, 2, 500, 7, 1, 1, 10, 1, "speed_x"),
    Channel("gyro_x", -16000, 16000, True, True, 1000, 2, 250, 7, 1, 1, 10, 1, "incline_x"),
    Channel("gyro_y", -16000, 16000, True, True, 1000, 2, 250, 7, 1, 1, 10, 1, "incline_y"),
    Channel("gyro_z", -16000, 16000, True, True, 1000, 2, 250, 7, 1, 1, 10, 1, "incline_z"),
    Channel("temperature", -500, 600, True, True, 5, 0, 2, 7, 1, 2, 15, 0),
    Channel("humidity", 0, 100, False, True, 5, 0, 2, 7, 1, 2, 15, 0),
    Channel("lux", 0, 130000, False, True, 20000, 0, 5000, 7, 1, 2, 15, 0),
    Channel("distance", 0, 20000, False, True, 100, 20, 25, 7, 1, 1, 10, 1, "approach_speed"),
)

CHANNEL_BY_NAME = {channel.name: channel for channel in CHANNELS}


# ---------------------------------------------------------------------------
# AXI 레지스터 디코드 (fpga_interface.build_input_words의 역변환)
# ---------------------------------------------------------------------------

def _signed(value: int, bits: int) -> int:
    value &= (1 << bits) - 1
    return value - (1 << bits) if value & (1 << (bits - 1)) else value


def decode_channels(words) -> dict:
    """PL이 실제로 보게 되는 정수 채널값을 레지스터에서 복원한다."""
    reg0, reg1, reg2, reg3, reg4, reg5, reg6, reg7, _reg8, _reg9 = words
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
        "speed_x": _signed((reg4 >> 16) & 0xFF, 8) << 6,
        "speed_y": _signed((reg4 >> 24) & 0xFF, 8) << 6,
        "distance": reg5 & 0x7FFF,
        # 압축 필드는 AXI 언패킹에서 다시 왼쪽으로 시프트된다.
        # sensor_input_v1_0_S00_AXI.v:
        #   approach_speed = $signed({slv_reg5[24:15], 3'b0})
        #   speed_x        = $signed({slv_reg4[23:16], 6'b0})
        #   speed_z        = $signed({slv_reg6[25:18], 6'b0})
        # 시프트를 빠뜨리면 PL이 보는 값보다 8배/64배 작아진다.
        "approach_speed": _signed((reg5 >> 15) & 0x3FF, 10) << 3,
        "humidity": (reg5 >> 25) & 0x7F,
        "lux": reg6 & 0x3FFFF,
        "temperature": _signed(reg7 & 0x7FF, 11),
    }


# ---------------------------------------------------------------------------
# each_sensor_check 재현
# ---------------------------------------------------------------------------

class CheckerModel:
    def __init__(self, channel: Channel):
        self.ch = channel
        self.range_cnt = 0
        self.jump_cnt = 0
        self.stuck_cnt = 0
        self.prev_value = None
        self.prev_delta = 0
        self.delta_history = []
        self.flip_history = []
        self.prev_delta_sign = 0
        self.faults = {"range": 0, "jump": 0, "stuck": 0, "noise": 0}

    def step(self, value: int, trig_value: int) -> None:
        ch = self.ch

        # ---- range ----
        raw_range = (
            (ch.use_min and value < ch.range_min)
            or (ch.use_max and value > ch.range_max)
        )
        if raw_range:
            self.range_cnt = (RN if self.range_cnt >= RN - RU
                              else self.range_cnt + RU)
        else:
            self.range_cnt = 0 if self.range_cnt < RD else self.range_cnt - RD
        if self.range_cnt >= RN:
            self.faults["range"] += 1

        # ---- preprocessor delta ----
        delta = 0 if self.prev_value is None else value - self.prev_value

        # ---- jump (2차 차분) ----
        second = delta - self.prev_delta
        if abs(second) > ch.jump_threshold:
            self.jump_cnt = (JN if self.jump_cnt >= JN - JU
                             else self.jump_cnt + JU)
        else:
            self.jump_cnt = 0 if self.jump_cnt < JD else self.jump_cnt - JD
        if self.jump_cnt >= JN:
            self.faults["jump"] += 1

        # ---- stuck ----
        if ch.channel_type_2 == 0:
            testable = True
        else:
            testable = abs(trig_value) >= ch.stuck_threshold

        if delta != 0:
            self.stuck_cnt = (0 if self.stuck_cnt < ch.stuck_d
                              else self.stuck_cnt - ch.stuck_d)
        elif testable:
            self.stuck_cnt = (ch.stuck_n if self.stuck_cnt >= ch.stuck_n - ch.stuck_u
                              else self.stuck_cnt + ch.stuck_u)
        if self.stuck_cnt >= ch.stuck_n:
            self.faults["stuck"] += 1

        # ---- noise ----
        sign = 1 if delta >= 0 else -1
        self.delta_history.append(abs(delta))
        self.flip_history.append(1 if sign != self.prev_delta_sign
                                 and self.prev_delta_sign != 0 else 0)
        if len(self.delta_history) > HISTORY:
            self.delta_history.pop(0)
            self.flip_history.pop(0)
        if len(self.delta_history) == HISTORY:
            if (sum(self.delta_history) > ch.noise_1 * HISTORY
                    and sum(self.flip_history) > ch.noise_2):
                self.faults["noise"] += 1

        self.prev_value = value
        self.prev_delta = delta
        self.prev_delta_sign = sign


# ---------------------------------------------------------------------------
# 주행 시나리오 (물리 단위)
# ---------------------------------------------------------------------------

class FakeSensor:
    """SensorManager가 노출하는 속성 중 패킹에 필요한 것만."""

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


def scenario_stopped(sensor, index):
    """완전 정지. 모든 채널이 상수인 최악의 stuck 조건."""
    return


def scenario_straight(sensor, index):
    """직선 정속 80 km/h. 요레이트 0, 종가속 0."""
    sensor.speed_x = 22.2
    sensor.incline_z = 0.0


def scenario_constant_curve(sensor, index):
    """일정 반경 선회. 요각속도가 상수라 gyro_z delta == 0이 이어진다.

    codex 보고서에서 gyro_z stuck이 101프레임 관측된 조건이다.
    """
    yaw_rate = 0.25                       # rad/s
    sensor.speed_x = 15.0
    sensor.gyro_z = yaw_rate
    sensor.accel_y = yaw_rate * 15.0
    # 요각(deg)은 실제로 계속 증가한다 -> gyro stuck이 testable해진다.
    sensor.incline_z = (index * yaw_rate * 57.2958 / SAMPLE_RATE_HZ) % 180.0


def scenario_accelerating(sensor, index):
    """정지 -> 가속."""
    t = index / SAMPLE_RATE_HZ
    sensor.speed_x = min(25.0, 2.5 * t)
    sensor.accel_x = 2.5 if sensor.speed_x < 25.0 else 0.0


SCENARIOS = (
    ("정지", scenario_stopped),
    ("직선 정속", scenario_straight),
    ("일정 반경 선회", scenario_constant_curve),
    ("가속", scenario_accelerating),
)


def lux_daynight(index: int) -> float:
    """sensor_manager와 동일하게 30프레임마다 한 번만 갱신되는 조도."""
    block = index // 30
    return 30000.0 + block * 850.0


def pack(sensor, seq: int):
    return build_input_words(
        sample_seq=seq,
        accel_xyz=(sensor.accel_x, sensor.accel_y, sensor.accel_z),
        gyro_xyz=(sensor.gyro_x, sensor.gyro_y, sensor.gyro_z),
        incline_xyz=(sensor.incline_x, sensor.incline_y, sensor.incline_z),
        speed_xyz=(sensor.speed_x, sensor.speed_y, sensor.speed_z),
        distance_m=sensor.distance,
        approach_speed_mps=sensor.approach_speed,
        temperature=sensor.temperature,
        humidity_pct=sensor.humidity,
        lux=sensor.lux,
        speed_limit_kmh=50.0, weather=0, rpm_level=1,
        accelerator=3, brake=0, steering_normalized=0.0,
        manual_mode=False, gear=1, headlight=False, hazard=False, situation=0,
    )


def run_scenario(apply_fn, noise_enabled: bool):
    noise = SensorNoiseModel(enabled=noise_enabled,
                             sample_rate_hz=SAMPLE_RATE_HZ)
    models = {channel.name: CheckerModel(channel) for channel in CHANNELS}
    sensor = FakeSensor()

    for index in range(SAMPLES):
        sensor.lux = lux_daynight(index)
        apply_fn(sensor, index)
        noise.apply(sensor)
        decoded = decode_channels(pack(sensor, index))
        for name, model in models.items():
            trig_name = model.ch.trig
            trig = decoded.get(trig_name, 0) if trig_name else 0
            if trig_name in {"incline_x", "incline_y", "incline_z", "speed_x"}:
                # trig_val_1은 원신호가 아니라 그 delta다.
                trig = decoded[trig_name] - getattr(model, "_prev_trig", decoded[trig_name])
                model._prev_trig = decoded[trig_name]
            model.step(decoded[name], trig)

    return models


# ---------------------------------------------------------------------------
# 개별 점검
# ---------------------------------------------------------------------------

def check_sentinel() -> list:
    """무표적 sentinel이 노이즈로 깨지지 않는지."""
    failures = []
    noise = SensorNoiseModel(enabled=True)
    sensor = FakeSensor()
    for _ in range(200):
        sensor.distance = 200.0
        sensor.approach_speed = 0.0
        noise.apply(sensor)
        decoded = decode_channels(pack(sensor, 0))
        if decoded["distance"] != 20000 or decoded["approach_speed"] != 0:
            failures.append(
                f"sentinel 파손: distance={decoded['distance']} "
                f"approach_speed={decoded['approach_speed']}"
            )
            break
    return failures


def check_distance_range_reachable() -> list:
    """200 m 초과 주입이 실제로 PL range fault를 만드는지."""
    failures = []
    sensor = FakeSensor()
    sensor.distance = 250.0
    sensor.approach_speed = 0.08
    decoded = decode_channels(pack(sensor, 0))
    if decoded["distance"] != 25000:
        failures.append(
            f"distance 250 m -> {decoded['distance']} (기대 25000)"
        )
    model = CheckerModel(CHANNEL_BY_NAME["distance"])
    for _ in range(10):
        model.step(decoded["distance"], 8)
    if model.faults["range"] == 0:
        failures.append("distance range fault가 확정되지 않음")

    # 정상 sentinel은 range fault가 아니어야 한다.
    clean = CheckerModel(CHANNEL_BY_NAME["distance"])
    for _ in range(10):
        clean.step(20000, 0)
    if clean.faults["range"] != 0:
        failures.append("sentinel 20000이 range fault로 오판됨")
    return failures


def check_fault_injection_still_works() -> list:
    """잡음이 고장 주입을 지우지 않고, 위험도 시험을 오염시키지도 않는지."""
    failures = []

    # (a) sensor stuck 주입 채널은 skip 대상이라 상수가 유지되어야 한다.
    noise = SensorNoiseModel(enabled=True)
    model = CheckerModel(CHANNEL_BY_NAME["temperature"])
    sensor = FakeSensor()
    for _ in range(60):
        sensor.temperature = 22.0        # stuck 주입이 덮어쓴 상수
        noise.apply(sensor, skip={"temperature"})
        decoded = decode_channels(pack(sensor, 0))
        model.step(decoded["temperature"], 0)
    if model.faults["stuck"] == 0:
        failures.append("temperature stuck 주입이 확정되지 않음 (잡음이 고장을 지움)")

    # (b) 노면 위험도 주입은 온습도를 상수로 덮어쓴다. skip 대상이 아니므로
    #     잡음이 다시 입혀져 stuck이 생기면 안 된다.
    noise = SensorNoiseModel(enabled=True)
    models = {name: CheckerModel(CHANNEL_BY_NAME[name])
              for name in ("temperature", "humidity")}
    for _ in range(200):
        sensor = FakeSensor()
        sensor.temperature = -8.0        # BLACK ICE 조건 (raw -80)
        sensor.humidity = 95.0
        noise.apply(sensor)
        decoded = decode_channels(pack(sensor, 0))
        for name, model in models.items():
            model.step(decoded[name], 0)
    for name, model in models.items():
        for check in ("stuck", "range", "jump", "noise"):
            if model.faults[check]:
                failures.append(
                    f"노면 위험도 시험 중 {name} {check} "
                    f"{model.faults[check]}프레임"
                )
    return failures


def check_road_surface_classification() -> list:
    """온도 스케일이 risk_types.sv의 노면 분류와 맞는지.

    risk_types.sv:
        temperature <= -50 && humidity >= 90  -> BLACK ICE
        temperature <=   0 && humidity >= 70  -> ICE
        humidity >= 70                        -> WET
        else                                  -> DRY
    """
    failures = []

    def classify(temp_c, humidity_pct):
        sensor = FakeSensor()
        sensor.temperature = temp_c
        sensor.humidity = humidity_pct
        decoded = decode_channels(pack(sensor, 0))
        temp, hum = decoded["temperature"], decoded["humidity"]
        if temp <= -50 and hum >= 90:
            name = "BLACK ICE"
        elif temp <= 0 and hum >= 70:
            name = "ICE"
        elif hum >= 70:
            name = "WET"
        else:
            name = "DRY"
        return name, temp, hum

    # (물리값, 습도, 기대 분류)
    cases = [
        (22.0, 30.0, "DRY"),        # CLEAR
        (12.0, 85.0, "WET"),        # wet 프리셋
        (-5.0, 85.0, "ICE"),        # ice 프리셋
        (-8.0, 95.0, "BLACK ICE"),  # black_ice 프리셋
        (-3.0, 80.0, "ICE"),        # SNOW 프리셋
        (15.0, 85.0, "WET"),        # RAIN 프리셋
    ]
    for temp_c, humidity_pct, expected in cases:
        actual, raw_t, raw_h = classify(temp_c, humidity_pct)
        if actual != expected:
            failures.append(
                f"{temp_c} degC / {humidity_pct}% -> {actual} (기대 {expected}, "
                f"raw temp={raw_t} hum={raw_h})"
            )

    # 스케일 자체 확인: 0.1 degC LSB
    sensor = FakeSensor()
    sensor.temperature = 22.0
    if decode_channels(pack(sensor, 0))["temperature"] != 220:
        failures.append("22.0 degC가 raw 220으로 인코딩되지 않음 (스케일 오류)")

    # 프리셋 값이 range 안에 있는지 (-500..600)
    for temp_c, _humidity, _expected in cases:
        sensor = FakeSensor()
        sensor.temperature = temp_c
        raw = decode_channels(pack(sensor, 0))["temperature"]
        if not (-500 <= raw <= 600):
            failures.append(f"{temp_c} degC -> raw {raw}가 range를 벗어남")

    return failures


def check_noise_within_rtl_range() -> list:
    """잡음이 어떤 채널도 RTL range 밖으로 밀어내지 않는지."""
    failures = []
    for name, apply_fn in SCENARIOS:
        models = run_scenario(apply_fn, noise_enabled=True)
        for channel_name, model in models.items():
            if model.faults["range"]:
                failures.append(
                    f"[{name}] {channel_name} range fault "
                    f"{model.faults['range']}프레임"
                )
    return failures


# ---------------------------------------------------------------------------

def main() -> int:
    # Windows 콘솔 기본 코드페이지에서 한글이 깨지지 않게 한다.
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except (AttributeError, OSError):
        pass

    print("=" * 74)
    print("센서 잡음 / AXI 패킹 검증  (CARLA·FPGA 보드 불필요)")
    print("=" * 74)

    passed = 0
    failures = []

    # 1. 잡음 OFF -> stuck 오탐이 재현되는가 (문제 진단의 근거)
    print("\n[1] 잡음 OFF: stuck 오탐 재현 확인")
    reproduced = []
    for name, apply_fn in SCENARIOS:
        models = run_scenario(apply_fn, noise_enabled=False)
        stuck = {n: m.faults["stuck"] for n, m in models.items()
                 if m.faults["stuck"]}
        print(f"    {name:12s} stuck 발생 채널: "
              f"{', '.join(f'{k}({v})' for k, v in stuck.items()) or '없음'}")
        reproduced.extend(stuck)
    if reproduced:
        print("    -> 잡음이 없을 때 stuck 오탐이 실제로 발생한다. 진단 확인.")
        passed += 1
    else:
        failures.append("[1] 잡음 OFF에서도 stuck이 재현되지 않음 (모델 오류 의심)")

    # 2. 잡음 ON -> 모든 검사기 오탐 0
    print("\n[2] 잡음 ON: 정상 주행 오탐 0 확인")
    for name, apply_fn in SCENARIOS:
        models = run_scenario(apply_fn, noise_enabled=True)
        total = {}
        for channel_name, model in models.items():
            for check, count in model.faults.items():
                if count:
                    total[f"{channel_name}:{check}"] = count
        status = "PASS" if not total else "FAIL"
        print(f"    {name:12s} {status}  "
              f"{', '.join(f'{k}={v}' for k, v in total.items()) or '오탐 없음'}")
        if total:
            failures.append(f"[2] {name}: {total}")
        else:
            passed += 1

    # 3. 잡음 여유 측정
    print("\n[3] 잡음 검사기 여유 (10표본 |delta| 합 / 부호반전)")
    models = run_scenario(scenario_stopped, noise_enabled=True)
    for channel_name in ("temperature", "humidity", "lux", "gyro_z", "accel_z"):
        model = models[channel_name]
        limit = model.ch.noise_1 * HISTORY
        actual = sum(model.delta_history)
        flips = sum(model.flip_history)
        margin = "inf" if actual == 0 else f"{limit / actual:.1f}x"
        print(f"    {channel_name:12s} delta합 {actual:6d} / 한계 {limit:6d} "
              f"(여유 {margin:>6s}), 부호반전 {flips}/10 (한계 7)")

    # 4. sentinel 보존
    print("\n[4] 무표적 sentinel 보존")
    result = check_sentinel()
    if result:
        failures.extend(result)
        print("    FAIL " + "; ".join(result))
    else:
        print("    PASS  distance=20000, approach_speed=0 유지")
        passed += 1

    # 5. distance range 고장 도달성
    print("\n[5] distance range 고장 도달성")
    result = check_distance_range_reachable()
    if result:
        failures.extend(result)
        print("    FAIL " + "; ".join(result))
    else:
        print("    PASS  250 m -> 25000 -> range fault 확정, sentinel은 정상")
        passed += 1

    # 6. RTL range 침범 없음
    print("\n[6] 잡음이 RTL range를 침범하지 않음")
    result = check_noise_within_rtl_range()
    if result:
        failures.extend(result)
        print("    FAIL " + "; ".join(result))
    else:
        print("    PASS  전 시나리오·전 채널 range fault 0")
        passed += 1

    # 7. 고장 주입/위험도 주입과의 상호작용
    print("\n[7] 고장 주입 보존 및 위험도 시험 비오염")
    result = check_fault_injection_still_works()
    if result:
        failures.extend(result)
        print("    FAIL " + "; ".join(result))
    else:
        print("    PASS  stuck 주입은 유지되고, 노면 위험도 시험은 오염 없음")
        passed += 1

    # 8. 온도 스케일 / 노면 분류
    print("\n[8] 온도 0.1 degC 스케일과 노면 분류 정합")
    result = check_road_surface_classification()
    if result:
        failures.extend(result)
        print("    FAIL " + "; ".join(result))
    else:
        print("    PASS  DRY/WET/ICE/BLACK ICE 전부 물리적으로 도달 가능, "
              "전 프리셋 range 내")
        passed += 1

    print("\n" + "=" * 74)
    if failures:
        print(f"결과: FAIL  ({len(failures)}건)")
        for item in failures:
            print(f"  - {item}")
        return 1
    print(f"결과: PASS  ({passed}개 항목 통과)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
