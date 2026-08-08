from results import RoadResult
from vehicle_command import VehicleCommand
from road_control import RoadControl
import utils


class RoadLogic:

    # ======================================================
    # Surface Type (온도·습도 기준, 날씨와 무관)
    # ======================================================

    SURFACE_DRY = 0
    SURFACE_WET = 1
    SURFACE_ICE = 2
    SURFACE_BLACK_ICE = 3

    # ======================================================
    # Road Shock
    # ======================================================

    SHOCK_NORMAL = 0
    SHOCK_ROUGH = 1
    SHOCK_SEVERE = 2
    SHOCK_EXTREME = 3

    SURFACE_GRADE_NAMES = ["DRY", "WET", "ICE", "BLACK_ICE"]
    SHOCK_GRADE_NAMES = ["NORMAL", "ROUGH", "SEVERE", "EXTREME"]

    def __init__(self):
        pass

    def update(self, environment, sensor):

        result = RoadResult()

        temperature = environment.temperature
        humidity = environment.humidity
        speed_limit = environment.speed_limit

        speed = sensor.speed
        accel_z = sensor.accel_z
        rpm = sensor.rpm

        surface = self.calculate_surface_type(temperature, humidity)
        surface_control = self.calculate_surface_control(
            surface, speed, speed_limit, rpm
        )

        if speed >= 30:
            shock = self.calculate_road_shock(accel_z)
        else:
            shock = self.SHOCK_NORMAL

        shock_control = self.calculate_shock_control(shock, speed, speed_limit)

        control = self.merge_controls(surface_control, shock_control)

        result.final_risk = control.risk
        result.throttle = control.throttle
        result.brake = control.brake
        result.steering = 12
        result.steering_rate_limit = control.steering_rate
        result.gear_down_request = control.gear_down
        result.speed_limit = control.speed_limit

        result.weather = environment.weather
        result.surface_type = surface
        result.road_shock = shock

        result.surface_grade = self.SURFACE_GRADE_NAMES[surface]
        result.shock_grade = self.SHOCK_GRADE_NAMES[shock]

        result.autonomous_control = (control.risk >= VehicleCommand.RISK_MEDIUM)

        return result

    # ======================================================

    def make_control(self, risk, throttle, steering_rate, speed_limit, gear_down, brake=0):
        control = RoadControl()
        control.risk = risk
        control.throttle = throttle
        control.steering_rate = steering_rate
        control.speed_limit = speed_limit
        control.gear_down = gear_down
        control.brake = brake
        return control

    # ======================================================
    # ㄱ: Surface (온도/습도 전용, Black Ice > Ice 우선)
    # ======================================================

    def calculate_surface_type(self, temperature, humidity):
        if temperature <= -5 and humidity >= 90:
            return self.SURFACE_BLACK_ICE
        if temperature <= 0 and humidity >= 70:
            return self.SURFACE_ICE
        if humidity >= 70:
            return self.SURFACE_WET
        return self.SURFACE_DRY

    def calculate_surface_control(self, surface, speed, speed_limit, rpm):

        if surface == self.SURFACE_DRY:
            return self.make_control(VehicleCommand.RISK_LOW, 10, 100, speed_limit, False)

        if surface == self.SURFACE_WET:
            threshold = speed_limit * 0.9
            throttle = utils.dynamic_throttle_cap(speed, threshold, 8)
            return self.make_control(VehicleCommand.RISK_LOW, throttle, 100, int(threshold), False)

        if surface == self.SURFACE_ICE:
            threshold = speed_limit * 0.7
            throttle = utils.dynamic_throttle_cap(speed, threshold, 6)
            gear_down = (speed > threshold) and (rpm <= utils.RPM_LEVEL_DOWNSHIFT_THRESHOLD)
            return self.make_control(VehicleCommand.RISK_MEDIUM, throttle, 100, int(threshold), gear_down)

        # SURFACE_BLACK_ICE
        threshold = speed_limit * 0.5
        throttle = utils.dynamic_throttle_cap(speed, threshold, 4)
        gear_down = (speed > threshold) and (rpm <= utils.RPM_LEVEL_DOWNSHIFT_THRESHOLD)
        return self.make_control(VehicleCommand.RISK_HIGH, throttle, 100, int(threshold), gear_down)

    # ======================================================
    # ㄴ: Shock
    # ======================================================

    def calculate_road_shock(self, accel_z):
        # CARLA accel_z includes gravity (~9.81 when parked). Subtract it to get actual shock.
        shock_g = abs(abs(accel_z) - 9.81)
        if shock_g < 2.0:
            return self.SHOCK_NORMAL
        elif shock_g < 4.0:
            return self.SHOCK_ROUGH
        elif shock_g < 6.0:
            return self.SHOCK_SEVERE
        return self.SHOCK_EXTREME

    def calculate_shock_control(self, shock, speed, speed_limit):

        if shock == self.SHOCK_NORMAL:
            return self.make_control(VehicleCommand.RISK_LOW, 10, 100, 999, False)

        if shock == self.SHOCK_ROUGH:
            threshold = speed_limit * 0.8
            brake = 2 if speed > threshold else 0
            return self.make_control(VehicleCommand.RISK_LOW, 9, 100, int(threshold), False, brake)

        if shock == self.SHOCK_SEVERE:
            threshold = speed_limit * 0.6
            brake = 2 if speed > threshold else 0
            return self.make_control(VehicleCommand.RISK_MEDIUM, 7, 100, int(threshold), False, brake)

        # SHOCK_EXTREME
        threshold = speed_limit * 0.5
        brake = 2 if speed > threshold else 0
        return self.make_control(VehicleCommand.RISK_HIGH, 5, 100, int(threshold), False, brake)

    # ======================================================
    # Merge
    # ======================================================

    def merge_controls(self, surface, shock):
        control = RoadControl()
        control.risk = max(surface.risk, shock.risk)
        control.throttle = min(surface.throttle, shock.throttle)
        control.steering_rate = min(surface.steering_rate, shock.steering_rate)
        control.speed_limit = min(surface.speed_limit, shock.speed_limit)
        control.gear_down = surface.gear_down or shock.gear_down
        control.brake = max(surface.brake, shock.brake)
        return control