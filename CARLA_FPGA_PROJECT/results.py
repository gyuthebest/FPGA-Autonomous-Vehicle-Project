"""
==========================================================
CARLA FPGA Autonomous Driving Project

Logic Result Classes

각 판단 로직의 결과를 저장하는 클래스
==========================================================
"""

from dataclasses import dataclass
from vehicle_command import VehicleCommand


# ==========================================================
# TTC
# ==========================================================

@dataclass
class TTCResult:
    final_risk: int = VehicleCommand.RISK_LOW

    throttle: int = 10
    brake: int = 0
    steering: int = 12
    steering_rate_limit: int = 100

    gear_down_request: bool = False
    speed_limit: int = 999

    headlight: bool = False
    hazard: bool = False

    manual_request: bool = False
    manual_mode: bool = False
    autonomous_control: bool = False
    emergency_stop: bool = False

    distance: float = 0.0
    closing_speed: float = 0.0
    ttc: float = 999.0

    stopping_distance: float = 0.0
    driver_delay_distance: float = 0.0
    braking_distance: float = 0.0

    rpm: int = 800
    distance_over_range: bool = False

# ==========================================================
# Posture
# ==========================================================

@dataclass
class PostureResult:
    final_risk: int = VehicleCommand.RISK_LOW

    throttle: int = 10
    brake: int = 0
    steering: int = 12
    steering_rate_limit: int = 100

    gear_down_request: bool = False
    speed_limit: int = 999

    headlight: bool = False
    hazard: bool = False

    manual_request: bool = False
    manual_mode: bool = False
    autonomous_control: bool = False
    emergency_stop: bool = False

    roll: float = 0.0
    pitch: float = 0.0
    roll_rate: float = 0.0
    yaw_rate: float = 0.0
    lateral_accel: float = 0.0

    # -----------------------------
    # FPGA Debug
    # -----------------------------

    roll_score: int = 0
    yaw_score: int = 0
    lateral_score: int = 0

    recommended_speed: int = 999

    roll_grade: str = "SAFE"
    yaw_grade: str = "SAFE"
    lateral_grade: str = "SAFE"


# ==========================================================
# Road
# ==========================================================

@dataclass
class RoadResult:
    final_risk: int = VehicleCommand.RISK_LOW

    throttle: int = 10
    brake: int = 0
    steering: int = 12
    steering_rate_limit: int = 100

    gear_down_request: bool = False
    speed_limit: int = 999

    headlight: bool = False
    hazard: bool = False

    manual_request: bool = False
    manual_mode: bool = False
    autonomous_control: bool = False
    emergency_stop: bool = False

    weather: int = 0
    surface_type: int = 0
    road_shock: int = 0

    surface_grade: str = "DRY"
    shock_grade: str = "NORMAL"

# ==========================================================
# Vision
# ==========================================================

@dataclass
class VisionResult:
    final_risk: int = VehicleCommand.RISK_LOW

    throttle: int = 10
    brake: int = 0
    steering: int = 12
    steering_rate_limit: int = 100

    gear_down_request: bool = False
    speed_limit: int = 999

    headlight: bool = False
    hazard: bool = False

    manual_request: bool = False
    manual_mode: bool = False
    autonomous_control: bool = False
    emergency_stop: bool = False

    visibility: float = 100.0
    lux: float = 0.0

    lux_grade: str = "BRIGHT"
    weather_grade: str = "CLEAR"