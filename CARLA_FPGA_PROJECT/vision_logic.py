"""
==========================================================
CARLA FPGA Autonomous Driving Project

Vision Logic - 시야 위험 (조도 ㄱ + 날씨 ㄴ)
==========================================================
"""

from dataclasses import dataclass

import utils
from results import VisionResult
from environment import Environment
from vehicle_command import VehicleCommand


@dataclass
class VisionControl:
    risk: int
    throttle: int
    speed_limit: int
    headlight: bool
    hazard: bool


class VisionLogic:

    WEATHER_GRADE_NAMES = ["CLEAR", "FOG", "RAIN", "SNOW"]

    def __init__(self):
        pass

    def update(self, environment, sensor):
        result = VisionResult()

        weather = environment.weather
        speed_limit = environment.speed_limit
        speed = sensor.speed
        lux = sensor.lux

        lux_control = self.calculate_lux_control(lux, speed, speed_limit)
        weather_control = self.calculate_weather_control(weather, speed, speed_limit)

        control = self.merge_controls(lux_control, weather_control)

        result.final_risk = control.risk
        result.throttle = control.throttle
        result.brake = 0
        result.steering_rate_limit = 100
        result.speed_limit = control.speed_limit
        result.headlight = control.headlight
        result.hazard = control.hazard
        result.autonomous_control = (control.risk >= VehicleCommand.RISK_MEDIUM)
        result.lux = lux

        result.lux_grade = self.calculate_lux_grade(lux)
        result.weather_grade = self.WEATHER_GRADE_NAMES[weather]

        return result

    # ======================================================
    # ㄱ: 조도
    # ======================================================

    def calculate_lux_control(self, lux, speed, speed_limit):

        if lux >= 20000:
            return VisionControl(VehicleCommand.RISK_LOW, 10, speed_limit, False, False)

        if lux >= 1000:
            return VisionControl(VehicleCommand.RISK_LOW, 10, speed_limit, True, False)

        if lux >= 50:
            return VisionControl(VehicleCommand.RISK_LOW, 10, speed_limit, True, False)

        # Very Dark (< 50 lux)
        threshold = speed_limit * 0.9
        throttle = utils.dynamic_throttle_cap(speed, threshold, 10)
        return VisionControl(VehicleCommand.RISK_MEDIUM, throttle, int(threshold), True, False)

    def calculate_lux_grade(self, lux):
        if lux >= 20000:
            return "BRIGHT"
        if lux >= 1000:
            return "DIM"
        if lux >= 50:
            return "DARK"
        return "VERY_DARK"

    # ======================================================
    # ㄴ: 날씨
    # ======================================================

    def calculate_weather_control(self, weather, speed, speed_limit):

        if weather == Environment.CLEAR:
            return VisionControl(VehicleCommand.RISK_LOW, 10, speed_limit, False, False)

        if weather == Environment.RAIN:
            threshold = speed_limit * 0.9
            throttle = utils.dynamic_throttle_cap(speed, threshold, 8)
            return VisionControl(VehicleCommand.RISK_LOW, throttle, int(threshold), True, False)

        if weather == Environment.FOG:
            threshold = speed_limit * 0.6
            throttle = utils.dynamic_throttle_cap(speed, threshold, 8)
            return VisionControl(VehicleCommand.RISK_MEDIUM, throttle, int(threshold), True, True)

        # SNOW
        threshold = speed_limit * 0.6
        throttle = utils.dynamic_throttle_cap(speed, threshold, 5)
        return VisionControl(VehicleCommand.RISK_MEDIUM, throttle, int(threshold), False, False)

    # ======================================================

    def merge_controls(self, lux_control, weather_control):
        return VisionControl(
            risk=max(lux_control.risk, weather_control.risk),
            throttle=min(lux_control.throttle, weather_control.throttle),
            speed_limit=min(lux_control.speed_limit, weather_control.speed_limit),
            headlight=(lux_control.headlight or weather_control.headlight),
            hazard=(lux_control.hazard or weather_control.hazard),
        )