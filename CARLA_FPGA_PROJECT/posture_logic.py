"""
==========================================================
CARLA FPGA Autonomous Driving Project

Posture Logic - 자세 위험 (Roll ㄱ / Yaw ㄴ / Lateral ㄷ)
==========================================================
"""

from dataclasses import dataclass
import utils

from results import PostureResult
from vehicle_command import VehicleCommand


@dataclass
class PostureControl:
    risk: int
    throttle: int
    steering_rate: int
    brake: int
    gear_down: bool


class PostureLogic:

    def __init__(self):
        pass

    def update(self, sensor):
        result = PostureResult()

        speed = sensor.speed
        accel_y = sensor.accel_y
        gyro_x = sensor.gyro_x
        gyro_z = sensor.gyro_z
        rpm = sensor.rpm

        if speed < 15:
            result.final_risk = VehicleCommand.RISK_LOW
            result.throttle = 10
            result.brake = 0
            result.steering = 12
            result.steering_rate_limit = 100
            result.roll = 0.0
            result.pitch = 0.0
            result.roll_rate = gyro_x
            result.yaw_rate = gyro_z
            result.lateral_accel = accel_y
            result.autonomous_control = False
            result.recommended_speed = 999
            result.roll_score = 0
            result.yaw_score = 0
            result.lateral_score = 0
            result.roll_grade = "SAFE"
            result.yaw_grade = "SAFE"
            result.lateral_grade = "SAFE"
            return result

        roll_control = self.calculate_roll_control(gyro_x)
        yaw_control = self.calculate_yaw_control(gyro_z, rpm)
        lateral_control = self.calculate_lateral_control(accel_y, rpm)

        control = self.merge_controls(roll_control, yaw_control, lateral_control)

        result.final_risk = control.risk
        result.throttle = control.throttle
        result.brake = control.brake
        result.steering = 12
        result.steering_rate_limit = control.steering_rate
        result.gear_down_request = control.gear_down

        result.roll = 0.0
        result.pitch = 0.0
        result.roll_rate = gyro_x
        result.yaw_rate = gyro_z
        result.lateral_accel = accel_y

        result.roll_score = 1 if abs(gyro_x) > 40.0 else 0
        result.yaw_score = 2 if abs(gyro_z) >= 60.0 else (1 if abs(gyro_z) >= 30.0 else 0)
        result.lateral_score = 2 if abs(accel_y) >= 7.84 else (1 if abs(accel_y) >= 4.9 else 0)

        result.roll_grade = self.calculate_roll_grade(gyro_x)
        result.yaw_grade = self.calculate_yaw_grade(gyro_z)
        result.lateral_grade = self.calculate_lateral_grade(accel_y)

        # 문서 기준: 자세 위험은 Speed 간섭 x
        result.recommended_speed = 999

        result.autonomous_control = (control.risk >= VehicleCommand.RISK_MEDIUM)

        return result

    # ======================================================
    # ㄱ: Roll (각속도 x)
    # ======================================================

    def calculate_roll_control(self, gyro_x):
        if abs(gyro_x) <= 40.0:
            return PostureControl(VehicleCommand.RISK_LOW, 10, 100, 0, False)
        return PostureControl(VehicleCommand.RISK_MEDIUM, 0, 50, 0, False)

    def calculate_roll_grade(self, gyro_x):
        return "SAFE" if abs(gyro_x) <= 40.0 else "DANGER"

    def calculate_yaw_grade(self, gyro_z):
        g = abs(gyro_z)
        if g < 30.0:
            return "SAFE"
        if g < 60.0:
            return "CAUTION"
        return "DANGER"

    def calculate_lateral_grade(self, accel_y):
        a = abs(accel_y)
        if a < 4.9:
            return "SAFE"
        if a < 7.84:
            return "CAUTION"
        return "DANGER"
    

    # ======================================================
    # ㄴ: Yaw (각속도 z)
    # ======================================================

    def calculate_yaw_control(self, gyro_z, rpm):
        g = abs(gyro_z)
        if g < 30.0:
            return PostureControl(VehicleCommand.RISK_LOW, 10, 100, 0, False)
        if g < 60.0:
            return PostureControl(VehicleCommand.RISK_LOW, 8, 70, 0, False)
        return PostureControl(VehicleCommand.RISK_HIGH, 0, 50, 0, rpm <= utils.RPM_LEVEL_DOWNSHIFT_THRESHOLD)

    # ======================================================
    # ㄷ: Lateral (가속도 y, 0.5g=4.9 / 0.8g=7.84 m/s^2)
    # ======================================================

    def calculate_lateral_control(self, accel_y, rpm):
        a = abs(accel_y)
        if a < 4.9:
            return PostureControl(VehicleCommand.RISK_LOW, 10, 100, 0, False)
        if a < 7.84:
            return PostureControl(VehicleCommand.RISK_LOW, 7, 80, 0, False)
        return PostureControl(VehicleCommand.RISK_HIGH, 0, 60, 1, rpm <= utils.RPM_LEVEL_DOWNSHIFT_THRESHOLD)

    # ======================================================

    def merge_controls(self, roll, yaw, lateral):
        return PostureControl(
            risk=max(roll.risk, yaw.risk, lateral.risk),
            throttle=min(roll.throttle, yaw.throttle, lateral.throttle),
            steering_rate=min(roll.steering_rate, yaw.steering_rate, lateral.steering_rate),
            brake=max(roll.brake, yaw.brake, lateral.brake),
            gear_down=(roll.gear_down or yaw.gear_down or lateral.gear_down),
        )