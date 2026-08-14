"""
==========================================================
CARLA FPGA Autonomous Driving Project

utils.py

공통 계산 함수
==========================================================
"""

import math


# ==========================================================
# SPEED
# ==========================================================

def mps_to_kmh(speed_mps: float) -> float:
    """m/s → km/h"""
    return speed_mps * 3.6


def kmh_to_mps(speed_kmh: float) -> float:
    """km/h → m/s"""
    return speed_kmh / 3.6


# ==========================================================
# LIMIT
# ==========================================================

def clamp(value: float, minimum: float, maximum: float) -> float:
    """값을 minimum ~ maximum 범위로 제한"""

    if value < minimum:
        return minimum

    if value > maximum:
        return maximum

    return value


# ==========================================================
# NORMALIZE
# ==========================================================

def normalize(
    value: float,
    input_min: float,
    input_max: float,
    output_min: float = 0.0,
    output_max: float = 1.0
) -> float:
    """
    선형 정규화
    """

    if input_max == input_min:
        return output_min

    ratio = (value - input_min) / (input_max - input_min)

    return output_min + ratio * (output_max - output_min)


# ==========================================================
# DISTANCE
# ==========================================================

def calculate_distance(x1, y1, x2, y2) -> float:
    """
    2D 거리 계산
    """

    return math.sqrt(
        (x2 - x1) ** 2 +
        (y2 - y1) ** 2
    )


# ==========================================================
# TTC
# ==========================================================

def calculate_ttc(
    distance_meter: float,
    relative_speed_mps: float
):
    """
    Time To Collision 계산

    상대속도가 0 이하이면 충돌하지 않는 것으로 간주
    """

    if relative_speed_mps <= 0:
        return float("inf")

    return distance_meter / relative_speed_mps


# ==========================================================
# FILTER
# ==========================================================

def low_pass_filter(
    previous_value: float,
    current_value: float,
    alpha: float
):
    """
    1차 Low Pass Filter
    alpha : 0~1
    """

    return (
        alpha * current_value +
        (1.0 - alpha) * previous_value
    )


def moving_average(values):
    """
    이동 평균
    """

    if len(values) == 0:
        return 0.0

    return sum(values) / len(values)


# ==========================================================
# ANGLE
# ==========================================================

def degree_to_radian(angle_deg: float):

    return math.radians(angle_deg)


def radian_to_degree(angle_rad: float):

    return math.degrees(angle_rad)


# ==========================================================
# SIGN
# ==========================================================

def sign(value: float):

    if value > 0:
        return 1

    if value < 0:
        return -1

    return 0

# ==========================================================
# RPM LEVEL (FPGA 2bit 신호: 0~3)
# ==========================================================

RPM_LEVEL_BOUNDS = (1999, 3999, 5999, 8000)

# RPM<=3000 조건에 해당하는 레벨 (Level 1: 2000~3999 구간에 3000이 포함됨)
RPM_LEVEL_DOWNSHIFT_THRESHOLD = 1


def rpm_to_level(rpm: float) -> int:
    """
    실제 RPM 값을 FPGA 전송용 2bit RPM 레벨(0~3)로 양자화한다.
    Level 0: 0~1999   Level 1: 2000~3999
    Level 2: 4000~5999 Level 3: 6000~8000
    """
    if rpm <= RPM_LEVEL_BOUNDS[0]:
        return 0
    elif rpm <= RPM_LEVEL_BOUNDS[1]:
        return 1
    elif rpm <= RPM_LEVEL_BOUNDS[2]:
        return 2
    return 3


# ==========================================================
# DISTANCE UNIT (FPGA 15bit 신호: cm)
# ==========================================================

def cm_to_m(distance_cm: float) -> float:
    return distance_cm / 100.0


def m_to_cm(distance_m: float) -> float:
    return distance_m * 100.0

# ==========================================================
# FPGA DISTANCE RANGE (15bit: 0~20000cm = 0~200m)
# ==========================================================

FPGA_MAX_DISTANCE_M = 200.0


def cap_distance_for_fpga(distance_m: float) -> float:
    """
    FPGA 15bit 거리 신호(0~20000cm=0~200m) 표현 범위에 맞춰
    200m를 넘는 값은 FPGA가 실제로 받게 될 최대값(200m=20000)으로 고정한다.
    """
    return min(distance_m, FPGA_MAX_DISTANCE_M)

# ==========================================================
# THROTTLE
# ==========================================================

def dynamic_throttle_cap(speed: float, threshold_speed: float, cap_throttle: int) -> int:
    """
    현재 속도가 threshold_speed를 초과하면 Accel Off(0),
    이하이면 cap_throttle 값으로 제한하여 주행.
    (노면/시야 위험 판단 공통 패턴)
    """
    if speed > threshold_speed:
        return 0
    return cap_throttle


# ==========================================================
# WAYPOINT HEADING (장애물 스폰 정렬)
# ==========================================================

MAX_SPAWN_HEADING_DEG = 25.0


def heading_error_deg(waypoint_yaw: float, reference_yaw: float) -> float:
    """두 heading 사이의 부호 없는 최소 각도차(0~180도)."""
    return abs(((float(waypoint_yaw) - float(reference_yaw) + 180.0) % 360.0) - 180.0)


def select_aligned_waypoint(
    waypoints,
    reference_yaw: float,
    max_heading_deg: float = MAX_SPAWN_HEADING_DEG,
    reject_junction: bool = True,
):
    """자차 진행 방향과 정렬된 waypoint만 고른다. 없으면 None.

    기존 코드는 `min(candidates, key=heading_error)`만 사용했다. min()은 후보가
    전부 교차로 가지여도 "가장 덜 어긋난" 것을 무조건 반환하므로, Town04
    인터체인지/분기점에서 진행 방향과 90도 어긋난 waypoint에 차량이 스폰되어
    앞차가 도로를 가로질러 서 있는 것처럼 보였다.

    허용 각도를 넘거나 교차로 내부인 waypoint는 후보에서 제외하고, 남는 후보가
    없으면 None을 돌려 호출부가 이번 시도를 건너뛰게 한다.
    """
    best = None
    best_error = None

    for waypoint in waypoints or ():
        if reject_junction and getattr(waypoint, "is_junction", False):
            continue
        error = heading_error_deg(
            waypoint.transform.rotation.yaw, reference_yaw
        )
        if error > max_heading_deg:
            continue
        if best_error is None or error < best_error:
            best = waypoint
            best_error = error

    return best