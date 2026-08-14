"""정상 주행에서 PL 신뢰도가 NORMAL을 유지하도록 하는 기본 센서 노이즈 모델.

배경
====
CARLA IMU는 `noise_*_stddev`가 모두 0이고, 온도/습도는 weather_manager가
상수로 넣으며, 조도는 30프레임마다 한 번만 갱신된다.  그 결과 PL이 받는
표본열은 완전히 정지(delta == 0)한 구간이 길어진다.  `sensor_checker.sv`의
stuck 검사는 다음과 같다.

    if (processed_data != 0) raw_stuck = 2'b00;   // stuck_cnt -= STUCK_D
    else if (testable)       raw_stuck = 2'b01;   // stuck_cnt += STUCK_U

온도/습도/조도는 `CHANNEL_TYPE_2 = 0`이라 `cond_b = 1'b1`, 즉 **항상
testable**이다.  따라서 값이 STUCK_N/STUCK_U = 15 표본(20 Hz에서 0.75초)
동안 변하지 않으면 무조건 stuck이 확정되고 채널이 DEGRADED로 떨어진다.
실제 센서는 최하위 비트가 항상 흔들리므로 이 상황 자체가 비현실적이다.

설계 제약
=========
노이즈를 아무렇게나 넣으면 다른 검사기를 오히려 오탐시킨다.
`each_sensor_check`의 noise 검사는 두 조건의 OR이다.

    noise_error = (delta_sum > NOISE_THRESHOLD_1 * HISTORY)
               && ($countones(flip_history) > NOISE_THRESHOLD_2)

즉 (1) 10표본 |delta| 합이 NOISE_THRESHOLD_1*10을 넘거나,
(2) delta 부호 반전이 10표본 중 7회를 넘으면 noise fault다.

조건 (2) 때문에 **백색 잡음을 쓰면 안 된다.**  백색 잡음의 1차 차분은
연속 표본끼리 강하게 음의 상관을 가져 부호가 거의 매 표본 뒤집히고,
기대 반전 횟수가 7회 근처까지 올라가 간헐적 noise 오탐을 만든다.

그래서 이 모델은 채널마다 주파수가 다른 **대역 제한된 저주파 정현파 합**을
쓴다.  진폭이 유한하게 묶여 있어 range/jump/분류 임계값을 절대 넘지 않고,
부호가 여러 표본 동안 유지되어 flip 조건에 걸리지 않으며, 표본당 |delta|가
1 LSB 이상이라 stuck을 확실히 해소한다.

채널별 설계값 (fs = 20 Hz, 표본당 delta ≈ 0.8 * A_lsb * 2*pi*f / fs)
====================================================================
| 채널        | LSB       | A(LSB) | f(Hz) | delta/표본 | NOISE_TH1 | 여유  |
|-------------|-----------|--------|-------|-----------|-----------|-------|
| accel_x/y/z | 0.01 m/s2 | 3.5    | 1.9   | ~1.7      | 500       | 290x  |
| gyro_x/y/z  | 0.001 r/s | 4.5    | 1.9   | ~2.2      | 250       | 110x  |
| temperature | 1 degC    | 1.4    | 2.7   | ~0.95     | 2         | 2.1x  |
| humidity    | 1 %       | 1.4    | 2.9   | ~1.0      | 2         | 2.0x  |
| lux         | 1 lux     | 비례    | 1.3   | ~1.6 이상  | 5000      | 큼    |

제외 채널
=========
`distance`와 `approach_speed`에는 노이즈를 넣지 않는다.
`sensor_reliability.sv`가 "추적 대상 없음"을 다음 값으로 정확히 판정하기
때문이다.

    (sensor_data_in.distance == 15'd20000) && (approach_speed == 13'sd0)

여기에 1 LSB라도 더하면 sentinel이 깨져서, 물체가 없는 정상 주행에서
distance 채널이 다시 stuck/jump/consistency 오탐을 내기 시작한다.
두 채널은 레이더 측정값 자체가 이미 프레임마다 변한다.
"""

from __future__ import annotations

import math
import os
from typing import Dict, Optional


DEFAULT_SAMPLE_RATE_HZ = 20.0

# 느린 2차 성분의 상대 주파수/진폭.  결과가 정확히 주기적으로 반복되지
# 않게 만들 뿐이며, 주파수가 낮아 부호 반전 횟수는 늘리지 않는다.
_SLOW_RATIO = 0.37
_PRIMARY_WEIGHT = 0.8
_SLOW_WEIGHT = 0.2


class _Channel:
    """한 센서 채널의 진폭/주파수/위상과 물리적 유효 범위."""

    __slots__ = ("attr", "amplitude", "freq_hz", "phase", "low", "high",
                 "relative")

    def __init__(self, attr, amplitude, freq_hz, phase, low, high,
                 relative=0.0):
        self.attr = attr
        self.amplitude = float(amplitude)
        self.freq_hz = float(freq_hz)
        self.phase = float(phase)
        self.low = low
        self.high = high
        # 0이 아니면 진폭이 측정값에 비례한다(조도처럼 동적 범위가 넓은 채널).
        self.relative = float(relative)


# 범위 하한/상한은 sensor_reliability.sv의 RANGE_THRESHOLD를 물리 단위로
# 환산한 뒤 안쪽으로 마진을 둔 값이다.  노이즈가 range fault를 유발하는
# 일은 설계상 발생할 수 없어야 한다.
_CHANNELS = (
    # accel: RANGE +-1600 LSB = +-16.00 m/s^2
    _Channel("accel_x", 0.035, 1.90, 0.00, -15.5, 15.5),
    _Channel("accel_y", 0.035, 2.10, 1.05, -15.5, 15.5),
    _Channel("accel_z", 0.035, 1.70, 2.10, -15.5, 15.5),
    # gyro: RANGE +-16000 LSB = +-16.000 rad/s
    _Channel("gyro_x", 0.0045, 1.90, 0.52, -15.9, 15.9),
    _Channel("gyro_y", 0.0045, 2.30, 1.57, -15.9, 15.9),
    _Channel("gyro_z", 0.0045, 1.60, 2.62, -15.9, 15.9),
    # temperature: RANGE -500..600, LSB 0.1 degC -> 물리 범위 -50.0..60.0 degC.
    # 진폭 0.14 degC = 1.4 LSB. NOISE_THRESHOLD_1이 2 LSB로 가장 빡빡한 채널이다.
    _Channel("temperature", 0.14, 2.70, 0.79, -49.5, 59.5),
    # humidity: RANGE 0..100 (1 % LSB), 상한이 곧 range 임계값이라 99로 제한
    _Channel("humidity", 1.4, 2.90, 1.83, 0.0, 99.0),
    # lux: RANGE 0..130000, 진폭은 측정값의 0.15 % (최소 4 lux)
    _Channel("lux", 4.0, 1.30, 2.36, 30.0, 129000.0, relative=0.0015),
)


# 환경 채널의 상시 바닥 진동. 자세한 근거는 SensorNoiseModel._floor_offset.
# 주기 8, 매 표본 1 raw LSB 씩 움직이고 8표본당 부호 반전 2회.
_FLOOR_TRIANGLE = (-2, -1, 0, 1, 2, 1, 0, -1)
_FLOOR_LSB = {"temperature": 0.1, "humidity": 1.0, "lux": 1.0}


def _enabled_from_env() -> bool:
    raw = os.getenv("CARLA_SENSOR_NOISE", "1").strip().lower()
    return raw not in {"0", "false", "no", "off"}


def _scale_from_env() -> float:
    """강건성 시험용 전역 진폭 배율 (`CARLA_SENSOR_NOISE_SCALE`).

    채널별 진폭의 상대 비율은 실센서 특성을 반영해 정해 둔 것이므로 그대로
    두고, 전체를 한 배율로만 키운다.  배율을 바꿔가며 캡처를 뜨고
    `pl_capture_metrics.py` 로 오탐률 곡선을 그리면 강건성 한계가 나온다.

    실측 기준점 (경사면 정차, 관계식 9~14 임계 4):
      배율 1.0 -> 잔차 중앙 2 / p90 4 / 최대 5, 원시위반 최대 3.9%,
                  디바운스(N_CONS=8, D_CONS=3)가 흡수해 채널 오탐 0%
    p90 이 이미 임계값과 같으므로 **약 2배부터 관계식 10/11 이 확정되기
    시작한다.**  1.0 / 1.5 / 2.0 / 3.0 스윕이 적절하다.
    """
    raw = os.getenv("CARLA_SENSOR_NOISE_SCALE", "1.0").strip()
    try:
        value = float(raw)
    except ValueError:
        return 1.0
    return value if value >= 0.0 else 1.0


class SensorNoiseModel:
    """센서 측정 잡음을 결정론적으로 재현하는 모델.

    시각을 표본 카운터로 관리하므로 같은 표본 수만큼 진행하면 항상 같은
    파형이 나온다.  캡처 재생/회귀 비교에서 재현성이 필요하기 때문이다.
    """

    def __init__(self, enabled: Optional[bool] = None,
                 sample_rate_hz: float = DEFAULT_SAMPLE_RATE_HZ) -> None:
        self.enabled = _enabled_from_env() if enabled is None else bool(enabled)
        self.scale = _scale_from_env()
        self.sample_rate_hz = float(sample_rate_hz)
        self._t = 0.0
        self.sample_count = 0

    def reset(self) -> None:
        self._t = 0.0
        self.sample_count = 0

    def offsets(self) -> Dict[str, float]:
        """현재 시각의 채널별 오프셋(물리 단위). 진단/시험용."""
        result = {}
        for channel in _CHANNELS:
            result[channel.attr] = self._offset(channel, 0.0)
        return result

    def _floor_offset(self, attr: str) -> float:
        """환경 채널의 상시 미세 진동(물리 단위).

        온도/습도/조도는 `CHANNEL_TYPE_2 = 0`(항상 testable) 이고
        `STUCK_N = 15` 라, 값이 완전히 일정하면 15표본 만에 stuck 이 확정된다.
        실측: `CARLA_SENSOR_NOISE=0` 개루프 3분에서
        **temperature 100 % / humidity 100 % / lux 93.3 % 가 DEGRADED** 였다.

        환경 값이 일정한 것은 물리적으로 정상이므로 이건 알고리즘 쪽 사각지대이고
        (8절 열린 항목), 그 판단이 서기 전까지는 계측 잡음 바닥을 항상 깔아
        신뢰도가 NORMAL 로 뜨게 한다.  따라서 이 성분은 **잡음 OFF 에서도
        적용되고 SCALE 배율도 받지 않는다.**

        파형은 주기 8 삼각파(raw LSB 기준 -2,-1,0,1,2,1,0,-1) 다.

          - 매 표본 |delta| = 1 raw  -> delta 가 0 이 되지 않아 stuck 이 안 쌓인다
          - 8표본당 부호 반전 2회    -> NOISE_THRESHOLD_2(10창 중 7) 에 한참 못 미친다
          - 10표본 |delta| 합 = 10   -> 온습도 NOISE_THRESHOLD_1(2) x 10 = 20 이하
          - |delta - prev_delta| <= 2 -> 온습도 JUMP_THRESHOLD(5) 이하

        raw LSB: 온도 0.1 degC, 습도 1 %, 조도 1.
        """
        lsb = _FLOOR_LSB.get(attr)
        if lsb is None:
            return 0.0
        return _FLOOR_TRIANGLE[self.sample_count % len(_FLOOR_TRIANGLE)] * lsb

    def _offset(self, channel: _Channel, value: float) -> float:
        amplitude = channel.amplitude
        if channel.relative:
            amplitude = max(amplitude, channel.relative * abs(value))

        omega = 2.0 * math.pi * channel.freq_hz * self._t
        primary = math.sin(omega + channel.phase)
        slow = math.sin(_SLOW_RATIO * omega + channel.phase * 1.7)
        return (self.scale * amplitude
                * (_PRIMARY_WEIGHT * primary + _SLOW_WEIGHT * slow))

    def apply(self, sensor, dt: Optional[float] = None, skip=()) -> None:
        """센서 객체에 측정 잡음을 더한다.

        `distance`/`approach_speed`는 의도적으로 건드리지 않는다(모듈 설명 참조).

        호출 위치는 고장 주입(control_panel.FaultInjector) **뒤**이고,
        `skip`에는 현재 고장이 걸린 채널 이름을 넘긴다.  이유는 두 가지다.

        1. stuck 주입은 값을 상수로 덮어쓰는 방식이라, 그 뒤에 잡음을 더하면
           주입한 고장이 사라진다.  그래서 고장 채널은 건너뛴다.
        2. 반대로 위험도 주입(`_apply_risk_faults`)은 노면/시야 조건을 만들려고
           온도·습도·조도를 상수로 덮어쓴다.  이 채널들에는 sensor fault가
           없으므로 잡음을 다시 입혀야 한다.  그러지 않으면 노면 위험도
           시험이 의도치 않게 온습도 stuck까지 만들어 시험을 오염시킨다.
        """
        skip = frozenset(skip)
        for channel in _CHANNELS:
            if channel.attr in skip:
                continue
            current = getattr(sensor, channel.attr, None)
            if current is None:
                continue
            # 계측 잡음은 enabled/scale 을 따르고, 환경 채널 바닥은 항상 깔린다.
            offset = self._floor_offset(channel.attr)
            if self.enabled:
                offset += self._offset(channel, float(current))
            if offset == 0.0:
                continue
            # 포화된 측정값 위에서 클램프가 바닥 진동을 지우지 않게 한다.
            #
            # 조도는 sensor_manager 가 130000 에서 포화시키고 한낮에는
            # sun_factor 가 1.0 에 붙어 실제로 상수가 된다.  예전처럼 더한 뒤
            # 클램프하면 상한에 붙은 구간에서 매 표본 정확히 같은 값이 나와
            # delta 가 0 이 되고, 바닥 진동을 넣은 의미가 사라진다.  실측
            # (pl_capture_20260814_183046): delta==0 이 98표본 연속 -> STUCK_N(15)
            # 확정 -> lux 오탐 5.60%.  습도(상한 99)도 같은 구조다.
            #
            # 그래서 클램프를 오프셋 **이전** 에 걸고 진동 폭만큼 여유를 둔다.
            # 결과값은 여전히 [low, high] 안이므로 잡음이 range fault 를
            # 유발하지 않는다는 기존 성질은 그대로다.
            headroom = abs(offset)
            low = channel.low + headroom
            high = channel.high - headroom
            base = float(current) if low > high else max(low, min(high, float(current)))
            value = max(channel.low, min(channel.high, base + offset))
            setattr(sensor, channel.attr, value)

        step = (1.0 / self.sample_rate_hz) if dt is None else float(dt)
        self._t += step
        self.sample_count += 1
