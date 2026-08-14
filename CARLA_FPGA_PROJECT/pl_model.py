"""PL 로직의 비트 단위 소프트웨어 모델 (golden reference model).

목적
====
"FPGA가 우리 로직을 그대로 반영했는가"를 검증하기 위한 기준 구현이다.
같은 REG0..REG9 입력을 넣었을 때 이 모델의 출력과 PL(시뮬레이션 또는 실보드)
출력이 **비트 단위로 일치**해야 한다.  불일치는 곧 다음 중 하나다.

- RTL이 의도한 알고리즘과 다르게 구현됨
- 합성/타이밍/AXI 전송 문제
- 이 모델이 사양을 잘못 옮김 (이 경우도 반드시 밝혀져야 한다)

주의: 이 모델은 `analyze_pl_trace.py`가 읽는 RTL 트레이스와 비교되지만,
**RTL을 그대로 베껴 쓴 것이 아니라** 각 신호의 정의(단위, 임계값, 디바운스
규칙)를 근거로 다시 구현했다.  RTL을 전사(transcribe)하면 같은 오해를 공유해
검증 가치가 사라진다.  임계값 상수만 RTL과 공유한다.

대응 관계
=========
| 모델 클래스        | RTL 파일                |
|--------------------|-------------------------|
| Preprocessor       | preprocessor.sv         |
| EachSensorCheck    | sensor_checker.sv       |
| ConsistencyCheck   | consistency_checker.sv  |
| SensorReliability  | sensor_reliability.sv   |
| RiskTypes          | risk_types.sv           |
| RiskControl(일부)  | risk_control.sv         |

현재 커버리지는 MODEL_COVERAGE 를 참조하라.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional


MODEL_COVERAGE = {
    "preprocessor delta (1차 차분)": "완전",
    "preprocessor pred (동역학 기준값)": "완전 (distance/approach_speed/accel/gyro)",
    "range / jump / stuck / noise / timeout": "완전",
    "consistency": "관계식 1~17 전부",
    "reliability 상태(NORMAL/DEGRADED/INVALID)": "완전",
    "risk_types 분류": "완전",
    "risk_control 제동/가속 중재": "제동 상한 블렌딩 + 가속 최소값 체인",
    "risk_control 유효 tier / HUD": "완전 (신뢰도 기반 tier 상향 + 바닥값)",
    "risk_control TD/MRM 타이머": "구현 (단 1초 틱은 실시간 기준이라 위상 근사)",
}


# ---------------------------------------------------------------------------
# 공통 상수 (sensor_reliability.sv 와 일치해야 한다)
# ---------------------------------------------------------------------------

RU, RD, RN = 1, 1, 3          # Range
JU, JD, JN = 6, 1, 18         # Jump
TU, TD_, TN = 1, 2, 10        # Timeout
U_CONS, D_CONS, N_CONS = 1, 3, 8

HISTORY_LEN = 10
DROP_N = 2

# (b) 지속 고착 격상.
# stuck 은 자기 consistency 를 마스킹하므로((a) 참조) (s && c) 로는 더 이상
# INVALID 가 될 수 없다.  그러면 완전히 얼어붙은 센서가 영원히 DEGRADED 에
# 머물러 안전 구멍이 된다.  그래서 고착이 확정 임계의 N 배만큼 지속되면
# stuck 단독으로 INVALID 로 올린다.
#   gyro/accel/distance      : STUCK_N=10 -> 30 표본 = 1.5 초
#   temperature/humidity/lux : STUCK_N=15 -> 45 표본 = 2.25 초
# 안전 파라미터이므로 실주행 근거가 모이면 조정한다.
STUCK_HARD_MULTIPLIER = 3

# 표본 주기 (20 Hz). PLModel.step() 이 시간을 흘릴 때 쓰는 기본값.
NOMINAL_SAMPLE_PERIOD_NS = 50_000_000

S_DIST, C_DIST = 40, 1
S_APSP, C_APSP = 1, 20
S_ACC, C_ACC = 1, 10
S_GYR, C_GYR = 1024, 3574

TH_DIST = 360
TH_APSP = 51
TH_ACC = 76                   # 관계식 3/4/5 : 동역학 기준값과의 허용 잔차
TH_ACC_STOP = 4               # 관계식 9/10/11 : 정지 시 중력 성분만 남는다
TH_ACC_TILT = 43              # 관계식 15/16 : 기울기 기반 상호 검증
TH_GYR = 7300                 # 2026-08-14 상향 (양자화 바닥 1787 고려)
TH_GYR_STOP = 4               # 관계식 12/13/14: 정지 시 각속도 ~ 0
TH_GYR_STEER = 120            # 관계식 17 : 조향 기반 예상 요각속도

# preprocessor.sv 의 기하/중력 상수
G_ACC = 981                   # 중력 가속도 (raw, LSB 0.01 m/s^2)
LUT_SH = 10                   # LUT 고정소수점 자릿수 (1.0 = 1024)
INCL_MAX = 3000               # LUT 입력 클램프 (30.00 deg)
STEER_MAX = 100
CLAMP_TH = 2990               # 관계식 마스크 1/3 : 기울기 클램프 근접
LVL_TH = 300                  # 관계식 마스크 2 : 수평 판정
YAW_TH = 2000                 # 관계식 마스크 2 : 요레이트 과대
SPD_MIN = 100                 # 관계식 마스크 3 : 저속 제외
M_WIN = 100                   # pred_distance 재초기화 주기
W_WIN = 2                     # 속도 차분 창 (표본)
MASK_20S_SAMPLES = 20 * 20    # 20 s hold @ 20 Hz

NORMAL, DEGRADED, INVALID = 0, 1, 2


def _clamp_sat(count: int, threshold: int, step: int) -> int:
    """codex가 추가한 포화 누적: threshold를 넘지 않는다."""
    return threshold if count >= threshold - step else count + step


def _decay(count: int, step: int) -> int:
    return 0 if count < step else count - step


def _trunc(value: int, bits: int) -> int:
    """signed [bits-1:0] 레지스터에 저장했을 때의 값.

    pred_data_t 의 각 필드는 폭이 정해져 있고, 계산 결과가 그 폭을 넘으면
    RTL은 wrap 한다.  기준값이 wrap 되면 정상 주행이 고장으로 판정되므로
    (pred_gyro_*_1 이 실제로 그랬다) 모델도 같은 폭으로 자른다.
    """
    mask = (1 << bits) - 1
    value &= mask
    return value - (1 << bits) if value & (1 << (bits - 1)) else value


# ---------------------------------------------------------------------------
# preprocessor.sv 의 고정소수점 LUT (1.0 = 1 << LUT_SH)
#   인덱스 1칸 = 1.28 deg (raw 128), 선형 보간
# ---------------------------------------------------------------------------

LUT_SIN = (0, 23, 46, 69, 91, 114, 137, 159, 182, 205, 227, 249, 271,
           293, 315, 337, 358, 380, 401, 422, 442, 463, 483, 503, 523)
LUT_COS = (1024, 1024, 1023, 1022, 1020, 1018, 1015, 1012, 1008, 1003,
           999, 993, 987, 981, 974, 967, 959, 951, 942, 933, 923, 913,
           903, 892, 880)
LUT_TAN = (0, 23, 46, 69, 92, 115, 138, 161, 185, 209, 233, 257, 281,
           306, 331, 357, 382, 409, 436, 463, 491, 519, 548, 578, 608)
LUT_STEER = (0, 184, 369, 556, 745, 939, 1137, 1341, 1552, 1772, 2002,
             2244, 2501, 2776)


def _lut_odd(table, angle: int) -> int:
    """홀함수(sin/tan) LUT 조회 + 선형 보간."""
    abs_a = min(abs(angle), INCL_MAX)
    idx, rem = abs_a >> 7, abs_a & 127
    if idx >= 24:
        val = table[24]
    else:
        val = table[idx] + (((table[idx + 1] - table[idx]) * rem) >> 7)
    return -val if angle < 0 else val


def get_sin(angle: int) -> int:
    return _lut_odd(LUT_SIN, angle)


def get_tan(angle: int) -> int:
    return _lut_odd(LUT_TAN, angle)


def get_cos(angle: int) -> int:
    """짝함수 : 부호를 되돌리지 않고, 감산 방향으로 보간한다."""
    abs_a = min(abs(angle), INCL_MAX)
    idx, rem = abs_a >> 7, abs_a & 127
    if idx >= 24:
        return LUT_COS[24]
    return LUT_COS[idx] - (((LUT_COS[idx] - LUT_COS[idx + 1]) * rem) >> 7)


def get_steer_lut(steering: int) -> int:
    abs_a = min(abs(steering), STEER_MAX)
    idx, rem = abs_a >> 3, abs_a & 7
    if idx >= 13:
        val = LUT_STEER[13]
    else:
        val = LUT_STEER[idx] + (((LUT_STEER[idx + 1] - LUT_STEER[idx]) * rem) >> 3)
    return -val if steering < 0 else val


# ---------------------------------------------------------------------------
# 채널 정의 : sensor_reliability.sv 의 each_sensor_check 인스턴스와 1:1
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class ChannelSpec:
    name: str
    range_min: int
    range_max: int
    use_min: bool
    use_max: bool
    jump_threshold: int
    stuck_threshold: int
    noise_1: int
    noise_2: int
    stuck_u: int
    stuck_d: int
    stuck_n: int
    channel_type_1: int           # jump mask 용 situation 분류
    channel_type_2: int           # stuck testable 조건
    trig: Optional[str] = None    # stuck trigger 신호 (type 2 는 1차 차분을 다시 차분)


CHANNEL_SPECS = (
    ChannelSpec("distance", 0, 20000, False, True, 100, 20, 25, 7, 1, 1, 10, 1, 1, "approach_speed"),
    # CHANNEL_TYPE_2 == 2 : cond_b = |delta_distance - prev_delta_distance| >= TH.
    # (이전 모델은 delta_approach_speed 를 썼는데 sensor_reliability.sv 의
    #  trig_val_1/2 는 delta_distance / prev delta_distance 다.)
    ChannelSpec("approach_speed", -4000, 4000, True, True, 10, 2, 3, 7, 1, 1, 10, 1, 2, "delta_distance"),
    ChannelSpec("accel_x", -1600, 1600, True, True, 2000, 2, 500, 7, 1, 1, 10, 0, 1, "delta_speed_x"),
    # 이전 모델은 accel_y/z 도 delta_speed_x 를 트리거로 썼다.  RTL 은 축별로
    # delta_speed_y / delta_speed_z 를 쓴다.
    ChannelSpec("accel_y", -1600, 1600, True, True, 2000, 2, 500, 7, 1, 1, 10, 0, 1, "delta_speed_y"),
    ChannelSpec("accel_z", -1600, 1600, True, True, 2000, 2, 500, 7, 1, 1, 10, 0, 1, "delta_speed_z"),
    ChannelSpec("gyro_x", -16000, 16000, True, True, 1000, 2, 250, 7, 1, 1, 10, 0, 1, "delta_incline_x"),
    ChannelSpec("gyro_y", -16000, 16000, True, True, 1000, 2, 250, 7, 1, 1, 10, 0, 1, "delta_incline_y"),
    ChannelSpec("gyro_z", -16000, 16000, True, True, 1000, 2, 250, 7, 1, 1, 10, 0, 1, "delta_incline_z"),
    ChannelSpec("temperature", -500, 600, True, True, 5, 0, 2, 7, 1, 2, 15, 2, 0),
    ChannelSpec("humidity", 0, 100, False, True, 5, 0, 2, 7, 1, 2, 15, 2, 0),
    ChannelSpec("lux", 0, 130000, False, True, 20000, 0, 5000, 7, 1, 2, 15, 2, 0),
)

CHANNEL_ORDER = tuple(spec.name for spec in CHANNEL_SPECS)
SPEC_BY_NAME = {spec.name: spec for spec in CHANNEL_SPECS}


# ---------------------------------------------------------------------------
# each_sensor_check
# ---------------------------------------------------------------------------

class EachSensorCheck:
    """sensor_checker.sv 한 인스턴스."""

    def __init__(self, spec: ChannelSpec):
        self.spec = spec
        self.range_cnt = 0
        self.jump_cnt = 0
        self.stuck_cnt = 0
        self.timeout_cnt = 0
        self.timeout_drop = 0
        self.timeout_confirm_hold = 0
        self.delta_history: List[int] = []
        self.flip_history: List[int] = []
        self.prev_delta_sign = 0

        self.range_error = False
        self.jump_error = False
        self.stuck_error = False
        self.stuck_hard = False   # 지속 고착
        self.jump_hard = False    # 지속 jump
        self.noise_hard = False   # 지속 noise
        self.noise_cnt = 0        # noise 는 조합 판정이라 지속 카운터를 따로 둔다
        self.noise_error = False
        self.timeout_error = False
        self.timeout_mask_1s = False
        self.timeout_mask_2s = False

    # -- timeout 은 valid 와 무관하게 주기적으로 증거를 쌓는다 --------------
    def tick_timeout(self, raw_timeout: bool, valid: bool) -> None:
        spec_n = TN
        if raw_timeout:
            self.timeout_drop = DROP_N
        elif valid and self.timeout_drop:
            self.timeout_drop -= 1

        if raw_timeout and self.timeout_cnt >= spec_n - TU:
            self.timeout_confirm_hold = DROP_N
        elif valid and self.timeout_confirm_hold:
            self.timeout_confirm_hold -= 1

        if raw_timeout:
            self.timeout_cnt = _clamp_sat(self.timeout_cnt, spec_n, TU)
        elif valid:
            self.timeout_cnt = _decay(self.timeout_cnt, TD_)

        self.timeout_error = (self.timeout_cnt >= spec_n
                              or self.timeout_confirm_hold != 0)
        self.timeout_mask_1s = self.timeout_drop == DROP_N
        self.timeout_mask_2s = self.timeout_drop != 0

    def step(self, value: int, delta: int, prev_delta: int,
             trig: int, situation: int, stuck_mask: bool,
             mask_1s: bool, mask_2s: bool) -> None:
        """mask_1s / mask_2s 는 **이번 표본을 처리하기 전** 의 timeout_drop 에서
        나온 값이어야 한다.  RTL 에서 tm1/tm2 는 레지스터 출력의 조합 논리이고,
        같은 클럭에서 검사기가 읽는 값은 감쇠 전 값이다.  감쇠 후 값을 쓰면
        복구 첫 표본(간격을 가로지르는 거대한 delta)이 noise 이력에 들어간다."""
        spec = self.spec

        # ---- range ----
        raw_range = ((spec.use_min and value < spec.range_min)
                     or (spec.use_max and value > spec.range_max))
        self.range_cnt = (_clamp_sat(self.range_cnt, RN, RU) if raw_range
                          else _decay(self.range_cnt, RD))
        self.range_error = self.range_cnt >= RN

        # ---- jump (2차 차분) ----
        jump_mask = ((spec.channel_type_1 == 1 and situation in (1, 2))
                     or (spec.channel_type_1 == 2 and situation == 3))
        if mask_2s:
            pass                                   # 증거 유지
        elif abs(delta - prev_delta) <= spec.jump_threshold:
            self.jump_cnt = _decay(self.jump_cnt, JD)
        elif not jump_mask:
            self.jump_cnt = _clamp_sat(self.jump_cnt, JN * STUCK_HARD_MULTIPLIER, JU)
        self.jump_error = self.jump_cnt >= JN
        self.jump_hard = self.jump_cnt >= JN * STUCK_HARD_MULTIPLIER

        # ---- stuck ----
        cond_b = True if spec.channel_type_2 == 0 else abs(trig) >= spec.stuck_threshold
        testable = cond_b and not stuck_mask
        # 포화 상한은 STUCK_N 그대로 둔다.  stuck 은 지속 격상 대상이 아니므로
        # 상한을 올릴 이유가 없고, 올리면 복구만 느려진다
        # (consistency 에서 같은 이유로 4.87% -> 5.48% 가 됐던 사례 참조).
        stuck_ceiling = spec.stuck_n
        if mask_1s:
            pass
        elif delta != 0:
            self.stuck_cnt = _decay(self.stuck_cnt, spec.stuck_d)
        elif testable:
            self.stuck_cnt = _clamp_sat(self.stuck_cnt, stuck_ceiling, spec.stuck_u)
        self.stuck_error = self.stuck_cnt >= spec.stuck_n
        self.stuck_hard = self.stuck_cnt >= stuck_ceiling
        # (a) 값이 안 변한 표본에서는 consistency 를 누적하지 않는다.
        # 확정된 stuck_error 가 아니라 원인 신호(delta == 0)에 거는 이유는
        # 경쟁 조건 때문이다. consistency 는 N_CONS=8 표본, stuck 은
        # STUCK_N=10 표본에 확정되므로 consistency 가 먼저 설 수 있다.
        self.frozen = (delta == 0)

        # ---- noise ----
        if not mask_1s:
            sign = 1 if delta >= 0 else -1
            self.delta_history.append(abs(delta))
            self.flip_history.append(
                1 if self.prev_delta_sign and sign != self.prev_delta_sign else 0
            )
            if len(self.delta_history) > HISTORY_LEN:
                self.delta_history.pop(0)
                self.flip_history.pop(0)
            self.prev_delta_sign = sign
        # 크기와 부호 반전을 AND 로 묶는다.  부호 반전만으로는 정상 IMU 와
        # 잡음 센서를 구분하지 못한다 (sensor_checker.sv 의 같은 식 참조).
        self.noise_error = (sum(self.delta_history) > spec.noise_1 * HISTORY_LEN
                            and sum(self.flip_history) > spec.noise_2)
        # noise 는 디바운스 카운터가 없는 조합 판정이라 지속 여부를 따로 센다.
        # 다른 검사와 같은 배수를 쓰되 기준은 noise 창 길이(10표본)다.
        noise_ceiling = HISTORY_LEN * STUCK_HARD_MULTIPLIER
        if self.noise_error:
            self.noise_cnt = min(self.noise_cnt + 1, noise_ceiling)
        else:
            self.noise_cnt = max(self.noise_cnt - 1, 0)
        self.noise_hard = self.noise_cnt >= noise_ceiling


# ---------------------------------------------------------------------------
# consistency_check
# ---------------------------------------------------------------------------

class ConsistencyCheck:
    """consistency_checker.sv 한 인스턴스."""

    def __init__(self, scale: int, threshold: int):
        self.scale = scale
        self.threshold = threshold
        self.count = 0
        self.error = False
        self.hard = False        # 확정 임계의 N 배만큼 지속

    def step(self, sensor_data: int, pred_data: int,
             timeout_mask_2s: bool, mask: bool) -> None:
        # 포화 상한은 N_CONS 그대로 둔다.  consistency 는 지속 격상 대상이
        # 아니므로(_resolve_states 참조) 상한을 올릴 이유가 없고, 올리면
        # 복구가 느려져 DEGRADED 구간만 길어진다.
        # 실측: 상한 24 로 올렸을 때 accel_x 4.87% -> 5.48%.
        ceiling = N_CONS
        if timeout_mask_2s:
            pass                                   # 증거 유지
        elif abs(sensor_data * self.scale - pred_data) <= self.threshold:
            self.count = _decay(self.count, D_CONS)
        elif not mask:
            self.count = _clamp_sat(self.count, ceiling, U_CONS)
        self.error = self.count >= N_CONS
        self.hard = self.count >= ceiling


# ---------------------------------------------------------------------------
# preprocessor
# ---------------------------------------------------------------------------

DELTA_FIELDS = (
    "distance", "approach_speed", "accel_x", "accel_y", "accel_z",
    "gyro_x", "gyro_y", "gyro_z", "temperature", "humidity", "lux",
    "speed_x", "speed_y", "speed_z", "incline_x", "incline_y", "incline_z",
)


class Preprocessor:
    """preprocessor.sv : 1차 차분과 동역학 기준값."""

    def __init__(self):
        self.prev: Dict[str, int] = {}
        self.delta: Dict[str, int] = {name: 0 for name in DELTA_FIELDS}
        self.prev_delta: Dict[str, int] = {name: 0 for name in DELTA_FIELDS}
        self.pred: Dict[str, int] = {name: 0 for name in (
            "distance", "approach_speed",
            "accel_x_1", "accel_x_2", "accel_x_3",
            "accel_y_1", "accel_y_2", "accel_y_3",
            "accel_z_1", "accel_z_2",
            "gyro_x", "gyro_x_2", "gyro_y", "gyro_y_2", "gyro_z", "gyro_z_2",
            "gyro_z_3",
        )}
        # 속도 지연선 : pred_accel_*_1 은 W(=2) 표본 전 속도와 비교한다.
        self.speed_hist = {axis: [0] * W_WIN for axis in ("x", "y", "z")}
        self.win = 0
        self.mask_1 = False
        self.mask_2 = False
        self.mask_3 = False
        self.first = True

    def step(self, sample: Dict[str, int],
             timeout_mask_1s: bool = False,
             cons_mask_1s_apsp: bool = False,
             cons_mask_20s_apsp: bool = False) -> None:
        # RTL은 첫 표본을 특별 취급하지 않는다.  delta = 현재 - sensor_data_out
        # 이고 sensor_data_out의 리셋값이 0이므로, 첫 표본의 delta는 값 자체다.
        # 이 때문에 첫 표본은 noise 창(10표본)이 비워질 때까지 진단에 영향을 준다.
        self.prev_delta = dict(self.delta)
        for name in DELTA_FIELDS:
            current = sample.get(name, 0)
            self.delta[name] = current - self.prev.get(name, 0)

        incline_x = sample.get("incline_x", 0)
        incline_y = sample.get("incline_y", 0)
        gyro_z = sample.get("gyro_z", 0)
        speed_x = sample.get("speed_x", 0)

        # -- consistency_mask_1/2/3 : 현재 표본의 기하 조건에서 바로 나온다 ----
        self.mask_1 = abs(incline_x) >= CLAMP_TH or abs(incline_y) >= CLAMP_TH
        self.mask_2 = (abs(incline_x) > LVL_TH or abs(incline_y) > LVL_TH
                       or abs(gyro_z) > YAW_TH)
        self.mask_3 = (abs(speed_x) < SPD_MIN
                       or abs(incline_x) >= CLAMP_TH or abs(incline_y) >= CLAMP_TH)

        # -- 중력 성분 (차체 좌표계) -----------------------------------------
        sin_x, cos_x, tan_x = get_sin(incline_x), get_cos(incline_x), get_tan(incline_x)
        sin_y, cos_y, tan_y = get_sin(incline_y), get_cos(incline_y), get_tan(incline_y)
        grav_x = (-(G_ACC * sin_y)) >> LUT_SH
        grav_y = (G_ACC * sin_x * cos_y) >> (2 * LUT_SH)
        grav_z = (G_ACC * cos_x * cos_y) >> (2 * LUT_SH)

        # -- |(accel_y, accel_z)| 의 시프트 근사 (관계식 16) -------------------
        abs_y, abs_z = abs(sample.get("accel_y", 0)), abs(sample.get("accel_z", 0))
        abs_max, abs_min = max(abs_y, abs_z), min(abs_y, abs_z)
        approx_sqrt = max(abs_max + (abs_min >> 3),
                          abs_max - (abs_max >> 3) + (abs_min >> 1))

        # -- 속도 지연선 : 갱신 전 값이 W 표본 전 속도다 -----------------------
        old = {axis: self.speed_hist[axis][W_WIN - 1] for axis in ("x", "y", "z")}

        # -- pred_distance : 재초기화 조건 (preprocessor.sv 528-531) -----------
        reload_pred = (timeout_mask_1s or cons_mask_1s_apsp or cons_mask_20s_apsp
                       or self.win == M_WIN - 1
                       or sample.get("situation", 0) == 1)
        if reload_pred:
            pred_distance = sample.get("distance", 0) * S_DIST
        else:
            pred_distance = (self.pred["distance"]
                             - (self.prev.get("approach_speed", 0)
                                + sample.get("approach_speed", 0)) * C_DIST)

        speed_y = sample.get("speed_y", 0)
        speed_z = sample.get("speed_z", 0)
        accel_z = sample.get("accel_z", 0)
        steer_lut_val = get_steer_lut(sample.get("steering", 0))

        # 저장 폭은 types_pkg.sv 의 pred_data_t 를 따른다.
        self.pred["distance"] = _trunc(pred_distance, 21)
        self.pred["approach_speed"] = _trunc(
            -(sample.get("distance", 0) - self.prev.get("distance", 0)) * C_APSP, 13)
        self.pred["accel_x_1"] = _trunc((speed_x - old["x"]) * C_ACC + grav_x * S_ACC, 12)
        self.pred["accel_x_2"] = _trunc(grav_x, 12)
        self.pred["accel_x_3"] = _trunc((-approx_sqrt * tan_y) >> LUT_SH, 12)
        # 관계식 4 : 차체 좌표계 횡가속도 = dv_y/dt + g_y + 구심항 v_x*w_z.
        # 구심항이 없으면 정상 선회(dv_y/dt ~ 0 인데 a_y != 0)가 고장이 된다.
        # 계수는 1/1000 이 정확하지만 RTL 은 곱셈기를 아끼려고 >>> LUT_SH
        # (1/1024, 2.4% 작음)를 쓴다.  모델도 같은 근사를 쓴다.
        centripetal = (speed_x * gyro_z) >> LUT_SH
        self.pred["accel_y_1"] = _trunc(
            (speed_y - old["y"]) * C_ACC + grav_y * S_ACC + centripetal, 12)
        self.pred["accel_y_2"] = _trunc(grav_y, 12)
        self.pred["accel_y_3"] = _trunc((accel_z * tan_x) >> LUT_SH, 12)
        self.pred["accel_z_1"] = _trunc((speed_z - old["z"]) * C_ACC + grav_z * S_ACC, 12)
        self.pred["accel_z_2"] = _trunc(grav_z, 12)
        self.pred["gyro_x"] = _trunc(self.delta["incline_x"] * C_GYR, 28)
        self.pred["gyro_y"] = _trunc(self.delta["incline_y"] * C_GYR, 28)
        # yaw 는 ±18000(±180.00 deg)에서 되감기므로 차분을 최단 경로로 접는다.
        # preprocessor.sv 의 delta_incline_z_wrapped 와 같아야 한다.
        d_incl_z = self.delta["incline_z"]
        if d_incl_z > 18000:
            d_incl_z -= 36000
        elif d_incl_z < -18000:
            d_incl_z += 36000
        self.pred["gyro_z"] = _trunc(d_incl_z * C_GYR, 28)
        self.pred["gyro_x_2"] = 0
        self.pred["gyro_y_2"] = 0
        self.pred["gyro_z_2"] = 0
        self.pred["gyro_z_3"] = _trunc((speed_x * steer_lut_val) >> LUT_SH, 16)

        for axis, value in (("x", speed_x), ("y", speed_y), ("z", speed_z)):
            self.speed_hist[axis] = [value] + self.speed_hist[axis][:W_WIN - 1]
        self.win = 0 if reload_pred else self.win + 1

        self.prev = dict(sample)
        self.first = False


# ---------------------------------------------------------------------------
# sensor_reliability
# ---------------------------------------------------------------------------

DISTANCE_SENTINEL = 20000


# 관계식 -> (채널, 스케일, 임계값, pred 키, 마스크 이름)
#   마스크 이름은 SensorReliability._masks() 가 만드는 사전의 키다.
CONSISTENCY_RELATIONS = (
    (1,  "distance",       S_DIST, TH_DIST,       "distance",       "m7"),
    (2,  "approach_speed", S_APSP, TH_APSP,       "approach_speed", "m8"),
    (3,  "accel_x",        S_ACC,  TH_ACC,        "accel_x_1",      "m1"),
    (9,  "accel_x",        S_ACC,  TH_ACC_STOP,   "accel_x_2",      "m4"),
    (16, "accel_x",        S_ACC,  TH_ACC_TILT,   "accel_x_3",      "m6"),
    (4,  "accel_y",        S_ACC,  TH_ACC,        "accel_y_1",      "m1"),
    (10, "accel_y",        S_ACC,  TH_ACC_STOP,   "accel_y_2",      "m4"),
    (15, "accel_y",        S_ACC,  TH_ACC_TILT,   "accel_y_3",      "m5"),
    (5,  "accel_z",        S_ACC,  TH_ACC,        "accel_z_1",      "m1"),
    (11, "accel_z",        S_ACC,  TH_ACC_STOP,   "accel_z_2",      "m4"),
    # 6/7/8 의 S 가 절반인 것은 센서 비교항이 중점의 '합' 이기 때문이다.
    # relation_sensor_value() 참조.
    (6,  "gyro_x",     S_GYR // 2, TH_GYR,        "gyro_x",         "m2"),
    (12, "gyro_x",         S_ACC,  TH_GYR_STOP,   "gyro_x_2",       "m4"),
    (7,  "gyro_y",     S_GYR // 2, TH_GYR,        "gyro_y",         "m2"),
    (13, "gyro_y",         S_ACC,  TH_GYR_STOP,   "gyro_y_2",       "m4"),
    (8,  "gyro_z",     S_GYR // 2, TH_GYR,        "gyro_z",         "m2"),
    (14, "gyro_z",         S_ACC,  TH_GYR_STOP,   "gyro_z_2",       "m4"),
    (17, "gyro_z",         S_ACC,  TH_GYR_STEER,  "gyro_z_3",       "m3"),
)

GYRO_MIDPOINT_RELATIONS = (6, 7, 8)
GYRO_SIGN_FLIP_RELATIONS = (6, 7)


def relation_sensor_value(number: int, channel: str, sample, delta) -> int:
    """관계식의 센서 비교항.  sensor_reliability.sv 의 gyro_cmp_* 와 같아야 한다.

    관계식 6/7/8 만 순시 gyro 대신 중점의 '합' (gyro[t] + gyro[t-1]) 을 쓴다.
    기준값 (incline[t]-incline[t-1])*C_GYR 이 구간 평균 각속도이기 때문이며,
    합을 그대로 쓰는 대신 S 를 절반으로 준다.
    gyro[t-1] = gyro[t] - delta_gyro 이므로 합 = 2*gyro[t] - delta_gyro 다.

    관계식 6/7 은 여기에 부호까지 뒤집는다.  CARLA rotation.roll/pitch 의
    부호 규약이 IMU gyro x/y 축과 반대이기 때문이다 (yaw 는 일치).
    """
    if number not in GYRO_MIDPOINT_RELATIONS:
        return sample.get(channel, 0)
    midpoint_sum = 2 * sample.get(channel, 0) - delta.get(channel, 0)
    return -midpoint_sum if number in GYRO_SIGN_FLIP_RELATIONS else midpoint_sum


# consistency_check 인스턴스의 비교 폭 (sensor_reliability.sv 의 WIDTH 파라미터).
# 센서값과 기준값 모두 이 폭의 signed 로 잘려서 비교된다.
RELATION_WIDTH = {1: 21, 2: 13, 3: 12, 9: 12, 16: 12, 4: 12, 10: 12, 15: 12,
                  5: 12, 11: 12, 6: 28, 12: 16, 7: 28, 13: 16, 8: 28, 14: 16,
                  17: 16}

# "구조적 판정" : RTL pack_ch 와 같은 구조.
# "고장 개수"   : 확정된 고장 종류를 세는 방식.
# 두 규칙은 range 단독(전자 INVALID / 후자 DEGRADED)과
# jump+noise(전자 DEGRADED / 후자 INVALID)에서 갈린다.
PACK_RULE_STRUCTURAL = "structural"
PACK_RULE_COUNT = "count"


class SensorReliability:
    """sensor_reliability.sv : 검사기 집합 + 상태 판정."""

    def __init__(self, pack_rule: str = PACK_RULE_STRUCTURAL):
        self.pack_rule = pack_rule
        self.checks = {spec.name: EachSensorCheck(spec) for spec in CHANNEL_SPECS}
        self.cons = {
            number: ConsistencyCheck(scale, threshold)
            for number, _ch, scale, threshold, _p, _m in CONSISTENCY_RELATIONS
        }
        # mask_20s : approach_speed consistency 가 확정되면 20 s 동안 유지된다.
        self.cons_mask_20s_apsp_cnt = 0
        self.timeout_phase = 0
        self.state: Dict[str, int] = {name: NORMAL for name in CHANNEL_ORDER}

    # -- 마스크 ------------------------------------------------------------
    def _untrusted(self, name: str) -> bool:
        """range|timeout|stuck|jump|noise (consistency 는 포함하지 않는다)."""
        check = self.checks[name]
        return (check.range_error or check.timeout_error or check.stuck_error
                or check.jump_error or check.noise_error)

    def _masks(self, sample, situation, pre_mask_1, pre_mask_2, pre_mask_3):
        """consistency_mask_1..8.

        4~8 은 직전 표본까지 확정된 고장 상태(untrusted_*)에서 나온다.
        RTL 에서 range_err 등이 레지스터 출력이므로 같은 클럭의 판정에는
        이전 표본 결과가 쓰인다.
        """
        # situation 000 이 '정지'이므로 (situation != 000) 은 "정지가 아니면
        # 마스크", 즉 정지 관계식(9~14)을 정지 상태에서만 켠다는 뜻이다.
        moving = situation != 0
        return {
            "m1": pre_mask_1,
            "m2": pre_mask_2,
            "m3": pre_mask_3,
            "m4": moving,
            "m5": moving or self._untrusted("accel_z"),
            "m6": moving or self._untrusted("accel_y") or self._untrusted("accel_z"),
            "m7": (self._untrusted("approach_speed")
                   or sample.get("distance", 0) == DISTANCE_SENTINEL),
            "m8": self._untrusted("distance"),
        }

    @property
    def timeout_mask_1s_any(self) -> bool:
        return any(check.timeout_mask_1s for check in self.checks.values())

    @property
    def cons_mask_1s_apsp(self) -> bool:
        return self.cons[2].error

    @property
    def cons_mask_20s_apsp(self) -> bool:
        return self.cons_mask_20s_apsp_cnt > 0

    # -- 한 표본 ------------------------------------------------------------
    def step(self, sample, delta, prev_delta, pred, situation, valid,
             phase_expired: bool,
             pre_mask_1: bool = False, pre_mask_2: bool = False,
             pre_mask_3: bool = False) -> None:
        # 마스크는 검사기 갱신 전 상태에서 뽑는다 (레지스터 출력).
        masks = self._masks(sample, situation, pre_mask_1, pre_mask_2, pre_mask_3)
        tm1 = {name: self.checks[name].timeout_mask_1s for name in CHANNEL_ORDER}
        tm2 = {name: self.checks[name].timeout_mask_2s for name in CHANNEL_ORDER}

        raw_timeout = (not valid) and phase_expired
        for check in self.checks.values():
            check.tick_timeout(raw_timeout, valid)

        if not valid:
            return

        for spec in CHANNEL_SPECS:
            check = self.checks[spec.name]
            trig = 0
            if spec.trig:
                if spec.trig.startswith("delta_"):
                    base = spec.trig[6:]
                    trig = delta.get(base, 0)
                    if spec.channel_type_2 == 2:
                        trig -= prev_delta.get(base, 0)
                else:
                    trig = sample.get(spec.trig, 0)
            check.step(
                value=sample.get(spec.name, 0),
                delta=delta.get(spec.name, 0),
                prev_delta=prev_delta.get(spec.name, 0),
                trig=trig, situation=situation, stuck_mask=False,
                mask_1s=tm1[spec.name], mask_2s=tm2[spec.name],
            )

        for number, channel, _scale, _th, pred_key, mask_key in CONSISTENCY_RELATIONS:
            width = RELATION_WIDTH[number]
            # consistency 누적은 막지 않는다.  중복 계수는 상태 판정 단계에서
            # 걸러낸다(_resolve_states 의 c_independent).  그래야 cons_err 비트가
            # 진단 정보로 남아 클럭 추적·로그에서 원인을 볼 수 있다.
            self.cons[number].step(
                sensor_data=_trunc(
                    relation_sensor_value(number, channel, sample, delta), width),
                pred_data=_trunc(pred.get(pred_key, 0), width),
                timeout_mask_2s=tm2[channel],
                mask=masks[mask_key],
            )

        # mask_20s : 확정 시 재장전, 아니면 감쇠
        if self.cons[2].error:
            self.cons_mask_20s_apsp_cnt = MASK_20S_SAMPLES
        elif self.cons_mask_20s_apsp_cnt:
            self.cons_mask_20s_apsp_cnt -= 1

        self._resolve_states(sample)

    def consistency_error(self, channel: str) -> bool:
        return any(self.cons[number].error
                   for number, ch, *_rest in CONSISTENCY_RELATIONS if ch == channel)

    def consistency_hard(self, channel: str) -> bool:
        """관계식 불일치가 확정 임계의 N 배만큼 지속됐나."""
        return any(self.cons[number].hard
                   for number, ch, *_rest in CONSISTENCY_RELATIONS if ch == channel)

    def _resolve_states(self, sample) -> None:
        for name in CHANNEL_ORDER:
            check = self.checks[name]
            r, j = check.range_error, check.jump_error
            s, n = check.stuck_error, check.noise_error
            t = check.timeout_error
            c = self.consistency_error(name)

            # 무표적 sentinel: distance 진단만 마스크한다 (전송 timeout 제외).
            # approach_speed 는 0 이 실제 측정값(접근 없음)이고 그 진단은
            # 그대로 의미가 있으므로 마스크하지 않는다.
            if (name == "distance"
                    and sample.get("distance") == DISTANCE_SENTINEL
                    and sample.get("approach_speed") == 0):
                r = j = s = n = c = False

            # ---- consistency 중복 계수 제거 --------------------------------
            # range/jump/stuck 이 서면 그 채널 값은 이미 손상된 것이고,
            # 기준값과의 잔차가 커지는 것은 논리적 필연이다.  그 상황의
            # consistency 는 새 증거가 아니라 같은 고장을 두 번 센 것이다.
            #
            # 반대로 다른 넷이 깨끗한데 consistency 만 서면, 그것은 넷이
            # 구조적으로 못 보는 고장(바이어스/스케일/드리프트/축 뒤바뀜)을
            # 잡은 것이다.  실주행 캡처에서 accel_x 가 정확히 이 경우였다
            # (range/jump/stuck/noise 0.00%, 관계식 3 만 4.9%).
            #
            # 그래서 consistency 는 다른 검사가 전부 깨끗할 때만 증거로 센다.
            c_independent = c and not (r or j or s or n or t)

            # ---- 소프트 고장의 지속 격상 ------------------------------------
            # 이전 설계는 (s && c) 를 INVALID 로 삼았으나, 그 전제인
            # "stuck 과 consistency 는 독립적 증거" 가 위와 같이 성립하지
            # 않는다.  전제가 틀린 항을 억지로 살리는 대신, 원리가 맞는
            # 규칙으로 바꾼다: **일시적 소프트 고장은 DEGRADED, 오래
            # 지속되면 INVALID.**  네 검사에 균일하게 적용한다.
            # stuck 은 DEGRADED 다.
            # 오르내림의 원인은 "stuck 이 DEGRADED 라서"가 아니라 consistency 가
            # 뒤따라 확정됐다 풀렸다 하면서 (s^c) 와 (s&&c) 사이를 오갔기
            # 때문이다.  위의 c_independent 로 그 연쇄를 끊었으므로 stuck 은
            # DEGRADED 에 안정적으로 머무른다.
            soft = j or n or s or c_independent
            # consistency 는 지속 격상에서 제외한다.
            #
            # 근거: 관계식 3(accel_x)은 속도를 2표본 차분해 가속도를 추정하는
            # 방식의 한계로 정상 주행에서도 4.9% 만성 오탐이 있다.  만성
            # 오탐에 지속 격상을 걸면 DEGRADED 오탐이 INVALID 오탐으로
            # 증폭된다.  실측: accel_x 4.87%(INVALID 0) -> 5.48%(INVALID 135).
            #
            # jump/noise/stuck 은 같은 캡처에서 확정률 0.00% 이므로 격상해도
            # 오탐이 늘지 않는다.  기준값 모델(관계식 3)이 고쳐지면 그때
            # consistency 도 격상 대상에 넣는다.
            # stuck 은 지속 격상 대상이 아니다.  사용자 결정에 따라 고착은
            # 지속되더라도 DEGRADED 로 유지한다.
            soft_hard = check.jump_hard or check.noise_hard

            if (name == "distance"
                    and sample.get("distance") == DISTANCE_SENTINEL
                    and sample.get("approach_speed") == 0):
                soft = False
                soft_hard = False

            if self.pack_rule == PACK_RULE_STRUCTURAL:
                if r or t or soft_hard:
                    self.state[name] = INVALID
                elif soft:
                    self.state[name] = DEGRADED
                else:
                    self.state[name] = NORMAL
            else:
                faults = sum((r, j, s, n, c))
                self.state[name] = (INVALID if (t or faults >= 2)
                                    else DEGRADED if faults == 1 else NORMAL)

    def bitmap(self, attr: str) -> int:
        value = 0
        for index, name in enumerate(CHANNEL_ORDER):
            if getattr(self.checks[name], attr):
                value |= 1 << index
        return value


# ---------------------------------------------------------------------------
# risk_types
# ---------------------------------------------------------------------------

ACCEL_0_5G, ACCEL_0_8G, ACCEL_1_0G, ACCEL_2_0G = 490, 784, 980, 1960
GYRO_30DEGS, GYRO_40DEGS, GYRO_60DEGS = 524, 698, 1047


@dataclass
class Risk:
    collision: int = 0
    road_A: int = 0
    road_B: int = 0
    vision_A: int = 0
    vision_B: int = 0
    posture_A: int = 0
    posture_B: int = 0
    posture_C: int = 0


def classify_risk(sample: Dict[str, int]) -> Risk:
    """risk_types.sv 의 조합 논리."""
    risk = Risk()

    distance = sample.get("distance", 0)
    closing = sample.get("approach_speed", 0)
    if closing <= 0:
        risk.collision = 0
    # Emergency 경계는 TTC 1.4초.  risk_types.sv 와 같은 시프트 근사를 쓴다.
    #   1 + 1/4 + 1/8 + 1/32 = 1.40625  (오차 +0.45 %)
    elif distance <= (closing + (closing >> 2) + (closing >> 3) + (closing >> 5)):
        risk.collision = 4
    elif distance <= closing << 1:
        risk.collision = 3
    elif distance <= (closing << 1) + closing:
        risk.collision = 2
    elif distance <= closing << 2:
        risk.collision = 1
    else:
        risk.collision = 0

    temperature = sample.get("temperature", 0)
    humidity = sample.get("humidity", 0)
    if temperature <= -50 and humidity >= 90:
        risk.road_A = 3
    elif temperature <= 0 and humidity >= 70:
        risk.road_A = 2
    elif humidity >= 70:
        risk.road_A = 1
    else:
        risk.road_A = 0

    net_accel_z = abs(sample.get("accel_z", 0) - 980)
    if sample.get("speed_x", 0) < 833:
        risk.road_B = 0
    elif net_accel_z >= ACCEL_2_0G:
        risk.road_B = 3
    elif net_accel_z >= ACCEL_1_0G:
        risk.road_B = 2
    elif net_accel_z >= ACCEL_0_5G:
        risk.road_B = 1
    else:
        risk.road_B = 0

    lux = sample.get("lux", 0)
    risk.vision_A = 0 if lux >= 20000 else 1 if lux >= 1000 else 2 if lux >= 50 else 3
    risk.vision_B = sample.get("weather", 0)

    risk.posture_A = 1 if abs(sample.get("gyro_x", 0)) >= GYRO_40DEGS else 0

    abs_gz = abs(sample.get("gyro_z", 0))
    risk.posture_B = 2 if abs_gz >= GYRO_60DEGS else 1 if abs_gz >= GYRO_30DEGS else 0

    abs_ay = abs(sample.get("accel_y", 0))
    risk.posture_C = 2 if abs_ay >= ACCEL_0_8G else 1 if abs_ay >= ACCEL_0_5G else 0

    return risk


# ---------------------------------------------------------------------------
# risk_control (제동 중재 부분)
# ---------------------------------------------------------------------------

BRAKE_CAP_ICE = 5
BRAKE_CAP_BLACK_ICE = 3
BRAKE_CAP_LATERAL = 5
BRAKE_CAP_NONE = 15


# ---------------------------------------------------------------------------
# risk_control : 신뢰도 기반 유효 tier + HUD + TD/MRM 타이머
#   risk_control.sv 의 calc_effective_tier / td_condition / 1초 타이머
# ---------------------------------------------------------------------------

# 위험도 요소 -> 그 요소를 만드는 채널들 (risk_control.sv 102-205행)
RISK_GROUP_CHANNELS = {
    "collision": ("distance", "approach_speed"),
    "road_A": ("temperature", "humidity"),
    "road_B": ("accel_z",),
    "vision_A": ("lux",),
    "posture_A": ("gyro_x",),
    "posture_B": ("gyro_z",),
    "posture_C": ("accel_y",),
    "pitch": ("gyro_y",),           # 제어 미사용, 경고에만
    "longitudinal": ("accel_x",),   # 제어 미사용, 경고에만
}

# calc_effective_tier 의 N (위험도 state 분류 개수)
RISK_GROUP_N = {"collision": 5, "road_A": 4, "road_B": 4, "vision_A": 4,
                "posture_A": 2, "posture_B": 3, "posture_C": 3}

# INVALID 일 때의 바닥 tier (N -> floor).
# 2026-08-15 부터 쓰이지 않는다.  INVALID 는 TD/MRM 이 담당하고
# risk_control 은 원시 tier 를 그대로 통과시킨다 (calc_effective_tier 참조).
# 되돌릴 수 있도록 값만 남겨 둔다.
INVALID_FLOOR = {2: 1, 3: 1, 4: 2, 5: 2}

# TD 발동 조건에 들어가는 그룹 (축소운행 불가)
TD_GROUPS = ("collision", "posture_A", "posture_B", "posture_C")

TD_IDLE = 11          # "TD 없음" 을 뜻하는 초기값 (파이썬 쪽에서 '-' 로 표시)


def group_reliability(state: Dict[str, int], group: str) -> int:
    """그룹을 구성하는 채널 중 최악을 그룹 신뢰도로 삼는다."""
    values = [state[name] for name in RISK_GROUP_CHANNELS[group]]
    if INVALID in values:
        return INVALID
    return DEGRADED if DEGRADED in values else NORMAL


def calc_effective_tier(raw: int, last_valid: int, rel_state: int, n: int) -> int:
    """risk_control.sv calc_effective_tier.

    NORMAL   : 원시 tier 그대로
    DEGRADED : 한 단계 올리되 N-2 를 넘지 않는다 (원시보다 낮아지지 않는다)
    INVALID  : 마지막 유효 tier 를 한 단계 올린 값과 바닥값 중 큰 쪽
    """
    if rel_state == DEGRADED:
        pre = min(raw + 1, n - 2)
        return max(raw, pre)
    # NORMAL  : 원시 tier 그대로.
    # INVALID : 원시 tier 그대로.  INVALID 는 TD/MRM 이 담당하므로
    #   risk_control 에서 별도의 상향이나 바닥값을 적용하지 않는다.
    #   이전에는 바닥값(N=5 이면 2)을 적용해, 센서가 INVALID 로 확정되는
    #   즉시 충돌 tier 가 2가 되어 제동이 걸렸고 TD 10초와 MRM 이 시작되기도
    #   전에 차가 멈췄다.  hud_warning / td_condition 은 영향받지 않는다.
    return raw


class RiskControl:
    """risk_control.sv 의 신뢰도 반영 단계.

    보드가 내보내는 risk 워드(read_reg9)는 **원시 tier 가 아니라 유효 tier**다.
    classify_risk() 만으로 보드를 대조하면 신뢰도가 떨어진 구간에서 전부
    불일치로 보인다 (2026-08-14 충돌 시나리오에서 실제로 그렇게 보였다).

    TD/MRM 타이머는 PL 클럭 기반 1초 주기라 20 Hz 표본과 동기가 아니다.
    그래서 표본 수가 아니라 **경과 실시간**으로 tick_second() 를 호출해야 한다.
    """

    def __init__(self):
        self.last_valid = {group: 0 for group in RISK_GROUP_N}
        self.td_remain_sec = TD_IDLE
        self.td_invalid_duration = 0
        self.td_locked = False
        self.eff: Dict[str, int] = {group: 0 for group in RISK_GROUP_N}
        self.eff["vision_B"] = 0
        self.hud_warning = False
        self.td_condition = False
        self.prev_steering = 0
        # risk_control.sv 의 can_downshift 는 리셋 후 1 로 시작해 이 설계에서는
        # 다시 0 이 되지 않는다 (cnt_05 재적재만 한다).  모델도 동일하게 둔다.
        self.can_downshift = True
        self.control = {}
        # MRM 다운시프트는 1회성이다 (arbitrate 참조).
        self.mrm_downshift_done = False

    # -- 매 표본 --------------------------------------------------------
    def step(self, risk: "Risk", state: Dict[str, int],
             manual_mode: bool = False) -> None:
        rel = {group: group_reliability(state, group)
               for group in RISK_GROUP_CHANNELS}

        raw = {"collision": risk.collision, "road_A": risk.road_A,
               "road_B": risk.road_B, "vision_A": risk.vision_A,
               "posture_A": risk.posture_A, "posture_B": risk.posture_B,
               "posture_C": risk.posture_C}

        for group, n in RISK_GROUP_N.items():
            self.eff[group] = calc_effective_tier(
                raw[group], self.last_valid[group], rel[group], n)
        self.eff["vision_B"] = risk.vision_B          # 통과 (신뢰도 미반영)

        # last_valid 는 **유효 tier** 를 저장한다 (원시 tier 가 아니다).
        for group in RISK_GROUP_N:
            if rel[group] != INVALID:
                self.last_valid[group] = self.eff[group]

        self.hud_warning = any(rel[g] == INVALID for g in RISK_GROUP_CHANNELS)
        self.td_condition = any(rel[g] == INVALID for g in TD_GROUPS)

        if manual_mode:                                # 수동 전환 시 타이머 초기화
            self.td_remain_sec = TD_IDLE
            self.td_invalid_duration = 0
            self.td_locked = False

    # -- 1초 경과 -------------------------------------------------------
    def tick_second(self) -> None:
        """risk_control.sv 687-735행. 모든 판정은 갱신 **전** 값으로 한다
        (non-blocking 대입이라 같은 클럭에서 서로의 새 값을 보지 못한다)."""
        duration = self.td_invalid_duration
        locked = self.td_locked
        remain = self.td_remain_sec
        condition = self.td_condition

        if condition:
            if duration < 5:
                self.td_invalid_duration = duration + 1
            if duration >= 4:                # 갱신 전 값으로 판정한다
                self.td_locked = True
        elif not locked:
            self.td_invalid_duration = 0

        if condition or locked:
            if remain == TD_IDLE:
                self.td_remain_sec = 10
            elif remain > 0:
                self.td_remain_sec = remain - 1
        elif not locked:
            self.td_remain_sec = TD_IDLE

    @property
    def mrm(self) -> bool:
        return self.td_remain_sec == 0

    @property
    def transition_demand(self) -> bool:
        return self.td_remain_sec <= 10

    # -- 최종 제어 중재 (risk_control.sv 305-655행) -----------------------
    def arbitrate(self, sim: Dict[str, int]) -> Dict[str, int]:
        """유효 tier 로부터 최종 제어 명령을 만든다.

        risk_control.sv 의 조합 논리를 그대로 옮겼다.  각 위험 요소가 자기
        몫의 제어값을 내고, 마지막에 가속은 최소값, 제동은 마찰 상한 블렌딩,
        제한속도는 최소값으로 중재한다.

        sim 은 PL 이 받은 sim_data_in (raw 정수) 이다.
        """
        acc_in = sim.get("accelerator", 0)
        brake_in = sim.get("brake", 0)
        steer_in = sim.get("steering", 0)
        gear_in = sim.get("gear", 0)
        rpm = sim.get("rpm", 0)
        speed_x = sim.get("speed_x", 0)
        limit = sim.get("speed_limit", 0)

        # risk_control.sv 294-298행. 정수 곱/시프트까지 동일해야 한다.
        spd90 = (limit * 922) >> 10
        spd80 = (limit * 819) >> 10
        spd70 = (limit * 717) >> 10
        spd60 = (limit * 614) >> 10
        spd50 = limit >> 1

        e = self.eff
        col_acc = road_A_acc = road_B_acc = acc_in
        vis_A_acc = vis_B_acc = acc_in
        pos_A_acc = pos_B_acc = pos_C_acc = acc_in
        col_brake = road_B_brake = brake_in
        col_gear = gear_in
        pos_A_steer = pos_B_steer = pos_C_steer = steer_in
        col_hazard = vis_B_hazard = False
        vis_A_head = vis_B_head = False
        road_A_lim = road_B_lim = vis_A_lim = vis_B_lim = limit

        steering_delta = steer_in - self.prev_steering
        # 충돌 tier 다운시프트: RPM <= 3999 (rpm_to_level 기준 level <= 1) 이고
        # 최소 기어가 1단이므로 gear_in > 1 일 때만 내린다.
        # 사양의 "0.5s delay 이후 1단 더" (tier 3/4) 는 RTL 과 함께 미구현이다.
        downshift = rpm <= 1 and self.can_downshift and gear_in > 1

        # --- 충돌 ---------------------------------------------------------
        col = e.get("collision", 0)
        if col == 1:
            col_acc = 0
        elif col == 2:
            col_acc = 0
            col_brake = 2 if speed_x <= 1111 else (3 if speed_x <= 2222 else 4)
            if downshift:
                col_gear = gear_in - 1
        elif col == 3:
            col_acc = 0
            col_brake = 4 if speed_x <= 1111 else (6 if speed_x <= 2222 else 8)
            if downshift:
                col_gear = gear_in - 1
            col_hazard = True
        elif col == 4:
            col_acc = 0
            col_brake = 10
            if downshift:
                col_gear = gear_in - 1
            col_hazard = True

        # --- 노면 (road_A) -------------------------------------------------
        for tier, cap, lim in ((1, 8, spd90), (2, 6, spd70), (3, 4, spd50)):
            if e.get("road_A", 0) == tier:
                if acc_in > cap:
                    road_A_acc = cap
                if speed_x > 0 and speed_x > lim:
                    road_A_acc = 0
                road_A_lim = lim

        # --- 노면 충격 (road_B) --------------------------------------------
        for tier, cap, lim in ((1, 9, spd80), (2, 7, spd60), (3, 5, spd50)):
            if e.get("road_B", 0) == tier:
                if acc_in > cap:
                    road_B_acc = cap
                if speed_x > 0 and speed_x > lim:
                    road_B_acc = 0
                road_B_brake = 2
                road_B_lim = lim

        # --- 시야 (조도) ----------------------------------------------------
        vis_a = e.get("vision_A", 0)
        if vis_a in (1, 2):
            vis_A_head = True
        elif vis_a == 3:
            if speed_x > 0 and speed_x > spd90:
                vis_A_acc = 0
            vis_A_head = True
            vis_A_lim = spd90

        # --- 시야 (날씨) — 신뢰도 미반영, 원시 tier 를 그대로 쓴다 -----------
        vis_b = e.get("vision_B", 0)
        if vis_b == 1:
            if acc_in > 8:
                vis_B_acc = 8
            if speed_x > 0 and speed_x > spd90:
                vis_B_acc = 0
            vis_B_lim = spd90
            vis_B_head = True
        elif vis_b == 2:
            if acc_in > 8:
                vis_B_acc = 8
            if speed_x > 0 and speed_x > spd70:
                vis_B_acc = 0
            vis_B_lim = spd70
            vis_B_head = True
            vis_B_hazard = True
        elif vis_b == 3:
            if acc_in > 5:
                vis_B_acc = 5
            if speed_x > 0 and speed_x > spd60:
                vis_B_acc = 0
            vis_B_lim = spd60
            vis_B_head = True

        # --- 자세 (roll / yaw / lateral) ------------------------------------
        def clamp_steer(limit_delta: int) -> int:
            if steering_delta > limit_delta:
                return self.prev_steering + limit_delta
            if steering_delta < -limit_delta:
                return self.prev_steering - limit_delta
            return steer_in

        if e.get("posture_A", 0) == 1:
            pos_A_acc = 0
            pos_A_steer = clamp_steer(100)

        pos_b = e.get("posture_B", 0)
        if pos_b == 1:
            if acc_in > 8:
                pos_B_acc = 8
            pos_B_steer = clamp_steer(140)
        elif pos_b == 2:
            pos_B_acc = 0
            pos_B_steer = clamp_steer(100)

        pos_c = e.get("posture_C", 0)
        if pos_c == 1:
            if acc_in > 7:
                pos_C_acc = 7
            pos_C_steer = clamp_steer(160)
        elif pos_c == 2:
            pos_C_acc = 0
            pos_C_steer = clamp_steer(120)

        # --- 중재 ------------------------------------------------------------
        final_acc = min(col_acc, road_A_acc, road_B_acc, vis_A_acc,
                        vis_B_acc, pos_A_acc, pos_B_acc, pos_C_acc)

        blended = blend_brake(e.get("road_A", 0), e.get("posture_C", 0),
                              col_brake, road_B_brake)
        final_brake = blended["final_brake"]

        # 조향: 현재 조향값에서 가장 적게 움직이는 후보를 고른다
        # (risk_control.sv 610-635행).
        diff_in = abs(steer_in - self.prev_steering)
        diff_a = abs(pos_A_steer - self.prev_steering)
        diff_b = abs(pos_B_steer - self.prev_steering)
        diff_c = abs(pos_C_steer - self.prev_steering)
        if diff_a <= diff_b:
            d1, s1 = diff_a, pos_A_steer
        else:
            d1, s1 = diff_b, pos_B_steer
        if diff_c <= diff_in:
            d2, s2 = diff_c, pos_C_steer
        else:
            d2, s2 = diff_in, steer_in
        final_steer = s1 if d1 <= d2 else s2

        final_limit = min(road_A_lim, road_B_lim, vis_A_lim, vis_B_lim)
        final_gear = min(col_gear, gear_in)
        final_head = vis_A_head or vis_B_head
        final_hazard = col_hazard or vis_B_hazard

        # --- MRM 오버라이드 -------------------------------------------------
        # 사양(UNECE R157):
        #   1. Hazard ON        2. Brake level 3      3. 기어 1단 내리기
        #   -> 1/2/3 을 동시에 수행. 그 결과 차량이 완전 정차하면
        #   4. 정차 유지        5. 수동 개입 시 manual mode 로 주행
        #
        # **3번은 1회성이다.**  MRM 이 서는 순간 한 번만 내리고, 그 뒤의 변속은
        # Python 자율주행 로직(속도에 맞는 기어 단수)에 맡긴다.  매 표본
        # gear-1 을 내보내면 "계속 내리라"는 뜻이 되어 사양과 다르다.
        if self.mrm:
            final_acc = 0
            final_brake = 3
            final_hazard = True
            if not self.mrm_downshift_done:
                # rpm 조건을 걸지 않는다.  사양은 "MRM 이면 1단 내린다" 이고,
                # 정차 과정에서 rpm 이 언제 조건을 만족할지에 의존하면
                # 다운시프트가 아예 일어나지 않을 수 있다.
                if final_gear > 0:
                    final_gear = final_gear - 1
                self.mrm_downshift_done = True
        else:
            self.mrm_downshift_done = False

        self.prev_steering = final_steer
        return {
            "accelerator": final_acc, "brake": final_brake,
            "steering": final_steer, "gear": final_gear,
            "speed_limit": final_limit,
            "headlight": final_head, "hazard": final_hazard,
        }

    def risk_word(self) -> int:
        """read_reg9 의 하위 16비트 (sensor_input_v1_0_S00_AXI.v 454행 순서)."""
        return ((self.eff["collision"] & 0x7)
                | ((self.eff["road_A"] & 0x3) << 3)
                | ((self.eff["road_B"] & 0x3) << 5)
                | ((self.eff["vision_A"] & 0x3) << 7)
                | ((self.eff["vision_B"] & 0x3) << 9)
                | ((self.eff["posture_A"] & 0x1) << 11)
                | ((self.eff["posture_B"] & 0x3) << 12)
                | ((self.eff["posture_C"] & 0x3) << 14))


def blend_brake(eff_road_A: int, eff_posture_C: int,
                col_brake: int, road_B_brake: int) -> Dict[str, int]:
    """risk_control.sv 의 마찰 비례 제동 블렌딩."""
    surface_cap = (BRAKE_CAP_ICE if eff_road_A == 2
                   else BRAKE_CAP_BLACK_ICE if eff_road_A == 3
                   else BRAKE_CAP_NONE)
    lateral_cap = BRAKE_CAP_LATERAL if eff_posture_C >= 2 else BRAKE_CAP_NONE
    cap = min(surface_cap, lateral_cap)
    requested = max(col_brake, road_B_brake)
    return {
        "surface_cap": surface_cap, "lateral_cap": lateral_cap,
        "brake_cap": cap, "requested_brake": requested,
        "final_brake": min(requested, cap),
    }


# ---------------------------------------------------------------------------
# 파이프라인 전체
# ---------------------------------------------------------------------------

@dataclass
class ModelOutput:
    sample_seq: int
    delta: Dict[str, int] = field(default_factory=dict)
    pred: Dict[str, int] = field(default_factory=dict)
    range_err: int = 0
    jump_err: int = 0
    stuck_err: int = 0
    noise_err: int = 0
    timeout_err: int = 0
    cons_gyro_z: int = 0
    cons_err: Dict[int, bool] = field(default_factory=dict)   # 관계식 번호별
    state: Dict[str, int] = field(default_factory=dict)
    risk: Risk = field(default_factory=Risk)              # 원시 tier
    eff_risk: Dict[str, int] = field(default_factory=dict)  # 신뢰도 반영 tier
    risk_word: int = 0                                     # read_reg9 하위 16비트
    hud_warning: bool = False
    transition_demand: bool = False
    mrm: bool = False
    td_remain_sec: int = TD_IDLE
    # 최종 제어 명령 (risk_control.sv sim_data_out). 보드의 REG13/REG14 와 대조한다.
    final_accelerator: int = 0
    final_brake: int = 0
    final_steering: int = 0
    final_gear: int = 0
    final_speed_limit: int = 0
    final_headlight: bool = False
    final_hazard: bool = False


class PLModel:
    """PL 전체 파이프라인의 소프트웨어 기준 구현."""

    def __init__(self, pack_rule: str = PACK_RULE_STRUCTURAL):
        self.pre = Preprocessor()
        self.rel = SensorReliability(pack_rule=pack_rule)
        self.ctl = RiskControl()
        self.sample_count = 0
        self._elapsed_ns = 0          # TD 타이머용 누적 실시간

    def advance_time(self, delta_ns: int) -> int:
        """경과 실시간을 넣으면 1초 틱을 필요한 횟수만큼 돌린다.

        risk_control.sv 의 sec_cnt 는 PL 클럭 기반 자유 카운터라 표본과 동기가
        아니다.  표본 수(20개=1초)로 세면 지터만큼 어긋나므로, 캡처에 기록된
        실제 송신 시각 차이를 넣어야 한다.  다만 보드 카운터의 **위상**까지는
        복원할 수 없어 ±1틱 오차는 남는다.
        """
        self._elapsed_ns += max(0, int(delta_ns))
        ticks = self._elapsed_ns // 1_000_000_000
        self._elapsed_ns -= ticks * 1_000_000_000
        for _ in range(ticks):
            self.ctl.tick_second()
        return ticks

    def tick_missing_sample(self) -> None:
        """표본이 오지 않은 채 timeout phase(100 ms)가 만료된 경우.

        `timeout_phase_cnt` 는 valid_s1 에 리셋되고 UPDATE_CLK_X2 에서 되감기는
        주기 카운터라, 표본이 끊긴 동안 **100 ms 마다 한 번** raw_timeout 이
        뜬다.  preprocessor 는 valid_s0=0 이라 delta/pred 를 갱신하지 않으므로
        여기서는 timeout 증거만 쌓인다.

        호스트 지터로 100 ms 를 넘긴 프레임이 실제로 발생하며, 이를 반영하지
        않으면 그 구간 이후 디바운스 위상이 보드와 어긋난다.
        """
        for check in self.rel.checks.values():
            check.tick_timeout(raw_timeout=True, valid=False)

    def step(self, sample: Dict[str, int], sample_seq: int = 0,
             situation: int = 0, delta_ns: Optional[int] = None) -> ModelOutput:
        # TD/MRM 타이머는 1초 실시간 기준이라 시간을 흘려 주어야 동작한다.
        # 예전에는 advance_time() 을 호출부가 따로 불러야 했고, 실제로는
        # compare_golden_vs_pl 만 부르고 있어서 나머지 도구(pl_capture_metrics,
        # board_smoke_test 등)에서는 TD 가 영원히 카운트다운하지 않았다.
        # 기본값으로 표본 주기(50 ms)를 흘리고, 실제 송신 시각이 있는
        # 호출부는 delta_ns 로 정확한 값을 넘긴다.
        self.advance_time(NOMINAL_SAMPLE_PERIOD_NS if delta_ns is None
                          else delta_ns)
        # preprocessor 가 쓰는 마스크는 직전 표본까지의 신뢰도 결과다.
        # (RTL 에서 sensor_reliability -> preprocessor 는 레지스터 경유 되먹임)
        self.pre.step(
            sample,
            timeout_mask_1s=self.rel.timeout_mask_1s_any,
            cons_mask_1s_apsp=self.rel.cons_mask_1s_apsp,
            cons_mask_20s_apsp=self.rel.cons_mask_20s_apsp,
        )
        self.rel.step(
            sample=sample, delta=self.pre.delta, prev_delta=self.pre.prev_delta,
            pred=self.pre.pred, situation=situation, valid=True,
            phase_expired=False,
            pre_mask_1=self.pre.mask_1, pre_mask_2=self.pre.mask_2,
            pre_mask_3=self.pre.mask_3,
        )
        risk = classify_risk(sample)
        self.ctl.step(risk, self.rel.state,
                      manual_mode=bool(sample.get("manual_mode", 0)))
        control = self.ctl.arbitrate(sample)
        self.sample_count += 1
        return ModelOutput(
            sample_seq=sample_seq,
            delta=dict(self.pre.delta), pred=dict(self.pre.pred),
            range_err=self.rel.bitmap("range_error"),
            jump_err=self.rel.bitmap("jump_error"),
            stuck_err=self.rel.bitmap("stuck_error"),
            noise_err=self.rel.bitmap("noise_error"),
            timeout_err=self.rel.bitmap("timeout_error"),
            # cons_err_gyro_z = {관계식17, 관계식14, 관계식8}
            cons_gyro_z=(int(self.rel.cons[8].error)
                         | (int(self.rel.cons[14].error) << 1)
                         | (int(self.rel.cons[17].error) << 2)),
            cons_err={number: check.error for number, check in self.rel.cons.items()},
            state=dict(self.rel.state),
            risk=risk,
            eff_risk=dict(self.ctl.eff),
            risk_word=self.ctl.risk_word(),
            hud_warning=self.ctl.hud_warning,
            transition_demand=self.ctl.transition_demand,
            mrm=self.ctl.mrm,
            td_remain_sec=self.ctl.td_remain_sec,
            final_accelerator=control["accelerator"],
            final_brake=control["brake"],
            final_steering=control["steering"],
            final_gear=control["gear"],
            final_speed_limit=control["speed_limit"],
            final_headlight=control["headlight"],
            final_hazard=control["hazard"],
        )
