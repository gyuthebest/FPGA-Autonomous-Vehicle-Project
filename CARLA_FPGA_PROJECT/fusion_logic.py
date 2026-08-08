from vehicle_command import VehicleCommand

class FusionLogic:

    def fuse(
        self,
        ttc=None,
        posture=None,
        road=None,
        vision=None
    ):
        command = VehicleCommand()
        command.speed_limit = 999

        command.throttle = VehicleCommand.MAX_THROTTLE
        command.brake = 0
        command.steering = VehicleCommand.CENTER_STEERING
        command.steering_rate_limit = 100

        # ==================================================
        # TTC
        # ==================================================

        if ttc is not None:
            command.throttle = self.fuse_throttle(command.throttle, ttc.throttle)
            command.steering = self.fuse_steering(command.steering, ttc.steering)
            command.steering_rate_limit = min(command.steering_rate_limit, ttc.steering_rate_limit)

            command.gear_down_request |= ttc.gear_down_request
            command.autonomous_control |= ttc.autonomous_control
            command.manual_request |= ttc.manual_request
            command.manual_mode |= ttc.manual_mode
            command.emergency_stop |= ttc.emergency_stop
            command.hazard = self.fuse_hazard(command.hazard, ttc.hazard)

            command.final_risk = max(command.final_risk, ttc.final_risk)

        # ==================================================
        # Posture
        # ==================================================

        if posture is not None:
            command.throttle = self.fuse_throttle(command.throttle, posture.throttle)
            command.gear_down_request |= posture.gear_down_request
            command.autonomous_control |= posture.autonomous_control
            command.manual_request |= posture.manual_request
            command.manual_mode |= posture.manual_mode
            command.emergency_stop |= posture.emergency_stop

            command.steering_rate_limit = min(command.steering_rate_limit, posture.steering_rate_limit)

            command.final_risk = max(command.final_risk, posture.final_risk)

        # ==================================================
        # Road
        # ==================================================

        if road is not None:
            command.throttle = self.fuse_throttle(command.throttle, road.throttle)
            command.steering = self.fuse_steering(command.steering, road.steering)
            command.speed_limit = self.fuse_speed_limit(command.speed_limit, road.speed_limit)
            command.steering_rate_limit = min(command.steering_rate_limit, road.steering_rate_limit)

            command.gear_down_request |= road.gear_down_request
            command.autonomous_control |= road.autonomous_control
            command.manual_request |= road.manual_request
            command.manual_mode |= road.manual_mode
            command.emergency_stop |= road.emergency_stop

            command.final_risk = max(command.final_risk, road.final_risk)

        # ==================================================
        # Vision
        # ==================================================

        if vision is not None:
            command.throttle = self.fuse_throttle(command.throttle, vision.throttle)
            command.headlight = self.fuse_headlight(command.headlight, vision.headlight)
            command.hazard = self.fuse_hazard(command.hazard, vision.hazard)
            command.speed_limit = self.fuse_speed_limit(command.speed_limit, vision.speed_limit)
            command.steering_rate_limit = min(command.steering_rate_limit, vision.steering_rate_limit)

            command.autonomous_control |= vision.autonomous_control
            command.manual_request |= vision.manual_request
            command.manual_mode |= vision.manual_mode
            command.emergency_stop |= vision.emergency_stop

            command.final_risk = max(command.final_risk, vision.final_risk)

        # ==================================================
        # Brake: 우선순위 기반 (1.TTC 2.자세 3.노면 4.시야)
        # ==================================================

        command.brake = self.fuse_brake_priority(ttc, posture, road, vision)

        # ==================================================
        # Emergency Override
        # ==================================================

        if command.emergency_stop:
            command.throttle = 0
            command.brake = 10

        return command

    # ======================================================

    def fuse_throttle(self, current, new):
        return min(current, new)

    # ======================================================

    def fuse_brake_priority(self, ttc, posture, road, vision):
        """
        우선순위(1.TTC 2.자세위험 3.노면위험 4.시야위험)에서
        가장 높은 위험(=먼저 요청이 있는) 모듈의 Brake 값을 그대로 사용한다.
        """
        if ttc is not None and ttc.brake > 0:
            return ttc.brake
        if posture is not None and posture.brake > 0:
            return posture.brake
        if road is not None and road.brake > 0:
            return road.brake
        if vision is not None and vision.brake > 0:
            return vision.brake
        return 0

    # ======================================================

    def fuse_steering(self, current, new):
        return new

    # ======================================================

    def fuse_speed_limit(self, current, new):
        return min(current, new)

    # ======================================================

    def fuse_headlight(self, current, new):
        return current or new

    # ======================================================

    def fuse_hazard(self, current, new):
        return current or new