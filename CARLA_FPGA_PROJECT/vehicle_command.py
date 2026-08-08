"""
==========================================================
Vehicle Command

ScenarioManager / FPGA가 생성하는 최종 제어 명령
VehicleController는 이 명령만 보고 CARLA를 제어한다.
==========================================================
"""

class VehicleCommand:

    # ======================================================
    # FPGA Command Range
    # ======================================================

    MAX_THROTTLE = 10
    MAX_BRAKE = 10

    MAX_STEERING = 24
    CENTER_STEERING = 12

    # ======================================================
    # Risk Level
    # ======================================================

    RISK_LOW = 0
    RISK_MEDIUM = 1
    RISK_HIGH = 2

    # ======================================================

    def __init__(self):
        self.reset()
        self.steering_rate_limit = 100

    # ======================================================

    def reset(self):
        """
        모든 명령을 기본값으로 초기화
        매 프레임 호출된다.
        """

        # --------------------------
        # Vehicle Control
        # --------------------------

        self.throttle = 0
        self.brake = 0

        self.steering = self.CENTER_STEERING
        self.steering_rate_limit = 100

        self.gear_down_request = False
        self.speed_limit = 999
        self.reverse = False

        # --------------------------
        # Vehicle State
        # --------------------------

        self.headlight = False
        self.hazard = False

        # --------------------------
        # Driving Mode
        # --------------------------

        self.manual_request = False
        self.manual_mode = False
        self.autonomous_control = True
        self.emergency_stop = False

        # --------------------------
        # Risk
        # --------------------------

        self.final_risk = self.RISK_LOW

    # ======================================================

    def __repr__(self):
        return (
            f"VehicleCommand("
            f"throttle={self.throttle}, "
            f"brake={self.brake}, "
            f"steering={self.steering}, "
            f"steering_rate_limit={self.steering_rate_limit}, "
            f"gear_down={self.gear_down_request}, "
            f"speed_limit={self.speed_limit}, "
            f"risk={self.final_risk})"
        )