"""
==========================================================
CARLA FPGA Autonomous Driving Project

TTC Logic

- Time To Collision
- Automatic Emergency Braking
- Down Shift FSM
==========================================================
"""

import time
import utils
from results import TTCResult
from vehicle_command import VehicleCommand

class TTCLogic:

    # ======================================================
    # Risk Level
    # ======================================================

    SAFE = 0
    CAUTION = 1
    DANGER = 2
    CRITICAL = 3
    EMERGENCY = 4

    # ======================================================
    # TTC Threshold (sec)
    # ======================================================

    TTC_SAFE = 4.0
    TTC_CAUTION = 3.0
    TTC_DANGER = 2.0
    TTC_CRITICAL = 1.4

    # ======================================================
    # Safety Margin
    # ======================================================

    SAFETY_MARGIN = 5.0

    # ======================================================
    # Vehicle Constant
    # ======================================================

    DRIVER_DELAY = 0.0          # Driver perception + reaction
    MAX_DECELERATION = 8.5      # 1.0 g

    # ======================================================

    def __init__(self):
        self.last_downshift_time = 0.0

    # ======================================================

    def update(self, sensor, perception):

        result = TTCResult()

        # --------------------------------------------------
        # Sensor Data
        # --------------------------------------------------

        distance = perception.front_distance
        relative_speed = perception.relative_speed   # m/s
        vehicle_speed = sensor.speed                 # km/h
        rpm = sensor.rpm

        # --------------------------------------------------
        # FPGA 거리 신호 범위 반영 (15bit, 0~20000cm = 0~200m)
        # FPGA는 거리만 받아 내부에서 TTC를 계산하므로,
        # 200m를 넘는 값은 FPGA가 실제로 받게 될 값(최대 200m)으로 고정한다.
        # --------------------------------------------------

        distance_capped = utils.cap_distance_for_fpga(distance)
        distance_over_range = distance >= utils.FPGA_MAX_DISTANCE_M

        # --------------------------------------------------
        # Unit Conversion
        # --------------------------------------------------

        distance_m = max(0.0, distance_capped - self.SAFETY_MARGIN)
        vehicle_speed_ms = vehicle_speed / 3.6
        closing_speed = relative_speed

        # --------------------------------------------------
        # TTC Calculation
        # --------------------------------------------------

        
        ttc = self.calculate_ttc(distance_m, closing_speed)

        driver_delay_distance = self.calculate_driver_delay_distance(vehicle_speed_ms)
        braking_distance = self.calculate_braking_distance(vehicle_speed_ms)
        stopping_distance = self.calculate_stopping_distance(
            driver_delay_distance,
            braking_distance
        )

        # --------------------------------------------------
        # Risk
        # --------------------------------------------------

        risk = self.determine_risk(ttc)

        # --------------------------------------------------
        # Control
        # --------------------------------------------------

        throttle = self.calculate_throttle(risk)
        brake = self.calculate_brake_level(risk, vehicle_speed)
        gear_down = self.should_downshift(risk, rpm)

        # --------------------------------------------------
        # Result
        # --------------------------------------------------

        if risk <= self.CAUTION:
            result.final_risk = VehicleCommand.RISK_LOW
        elif risk == self.DANGER:
            result.final_risk = VehicleCommand.RISK_MEDIUM
        else:
            result.final_risk = VehicleCommand.RISK_HIGH

        result.throttle = throttle
        result.brake = brake
        result.rpm = rpm
        result.risk = risk
        result.gear_down_request = gear_down

        result.distance = distance_capped
        result.distance_over_range = distance_over_range
        result.closing_speed = closing_speed
        result.ttc = min(ttc, 20.0)  # 표시/로깅용 클램핑. 위험도 판정은 원본 ttc(raw) 기준으로 이미 완료됨

        result.driver_delay_distance = driver_delay_distance
        result.braking_distance = braking_distance
        result.stopping_distance = stopping_distance

        result.steering_rate_limit = 100

        # FPGA가 실제 제어권을 가져가는지 여부
        result.autonomous_control = (risk >= self.DANGER)
        result.emergency_stop = (risk == self.EMERGENCY)
        result.hazard = (risk >= self.CRITICAL)

        return result

    # ======================================================

    def calculate_closing_speed(
        self,
        relative_speed
    ):
        """
        상대속도(m/s)를 그대로 사용한다.
        """

        return max(
            relative_speed,
            0.0
        )

    # ======================================================

    def calculate_ttc(self, distance_m, closing_speed):
        """
        TTC 계산
        차량이 정지 상태이면 충돌하지 않으므로 매우 큰 값을 반환한다.
        """
        if closing_speed <= 0.01:
            return 999.0
        return distance_m / closing_speed

    # ======================================================

    def calculate_driver_delay_distance(self, vehicle_speed_ms):
        """
        운전자 인지 + 반응거리
        """
        return vehicle_speed_ms * self.DRIVER_DELAY

    # ======================================================

    def calculate_braking_distance(self, vehicle_speed_ms):
        """
        제동거리
        d = v² / (2a)
        """
        if vehicle_speed_ms <= 0:
            return 0.0
        return (vehicle_speed_ms ** 2) / (2 * self.MAX_DECELERATION)

    # ======================================================

    def calculate_stopping_distance(self, driver_delay_distance, braking_distance):
        """
        총 정지거리
        """
        return driver_delay_distance + braking_distance

    # ======================================================

    def determine_risk(self, ttc):
        """
        TTC 기반 위험도 판정 (문서 기준: TTC 값만으로 판정)
        """
        if ttc > self.TTC_SAFE:
            return self.SAFE
        elif ttc > self.TTC_CAUTION:
            return self.CAUTION
        elif ttc > self.TTC_DANGER:
            return self.DANGER
        elif ttc > self.TTC_CRITICAL:
            return self.CRITICAL
        return self.EMERGENCY

    # ======================================================

    def calculate_throttle(self, risk):
        if risk == self.SAFE:
            return 10
        return 0

    # ======================================================

    def calculate_brake_level(self, risk, speed_kmh):
        if risk in (self.SAFE, self.CAUTION):
            return 0

        if risk == self.DANGER:
            if speed_kmh <= 40:
                return 2
            elif speed_kmh <= 80:
                return 3
            else:
                return 4

        if risk == self.CRITICAL:
            if speed_kmh <= 40:
                return 4
            elif speed_kmh <= 80:
                return 6
            else:
                return 8

        return 10

    # ======================================================

    def should_downshift(self, risk, rpm):
        current_time = time.time()

        if risk == self.DANGER:
            return rpm <= utils.RPM_LEVEL_DOWNSHIFT_THRESHOLD

        # Critical / Emergency: RPM 레벨이 임계 이하이면 다운시프트, 0.5s 뒤 재요청
        if risk in (self.CRITICAL, self.EMERGENCY):
            if rpm <= utils.RPM_LEVEL_DOWNSHIFT_THRESHOLD and (current_time - self.last_downshift_time >= 0.5):
                self.last_downshift_time = current_time
                return True
            return False

        return False