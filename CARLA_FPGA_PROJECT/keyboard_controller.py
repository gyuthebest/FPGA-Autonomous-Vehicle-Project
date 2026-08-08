"""
==========================================================
CARLA FPGA Autonomous Driving Project

Keyboard Controller

- 시스템 이벤트(QUIT / ESC / R / M) 처리
- Manual Mode 주행 명령 생성 (전진/후진 상태머신 포함)
==========================================================
"""

import pygame

from vehicle_command import VehicleCommand


class KeyboardController:

    def __init__(self):

        self.manual_mode = False

        # "FORWARD": W=가속, S=브레이크(정지 유지 시 후진 전환)
        # "REVERSE": S=후진가속, W=브레이크(정지 유지 시 전진 복귀)
        self.drive_mode = "FORWARD"

    # ======================================================
    # System Events
    # ======================================================

    def poll_system_events(self):
        """
        반환값: 'run' | 'restart' | 'quit'
        pygame 이벤트 큐는 프레임당 한 번, 이 함수에서만 소비한다.
        """

        for event in pygame.event.get():

            if event.type == pygame.QUIT:
                return "quit"

            if event.type == pygame.KEYDOWN:

                if event.key == pygame.K_ESCAPE:
                    return "quit"

                if event.key == pygame.K_r:
                    return "restart"

                if event.key == pygame.K_m:
                    self.manual_mode = not self.manual_mode
                    print("[MODE]", "MANUAL" if self.manual_mode else "AUTO")

        return "run"

    # ======================================================
    # Driving Command (Manual Mode)
    # ======================================================

    def update(self, command, vehicle_speed_kmh):

        command.manual_mode = self.manual_mode
        command.autonomous_control = (not self.manual_mode)

        if not self.manual_mode:
            return

        keys = pygame.key.get_pressed()

        w = keys[pygame.K_w]
        s = keys[pygame.K_s]

        almost_stopped = vehicle_speed_kmh < 2.0

        if self.drive_mode == "FORWARD":

            if w:
                command.throttle = VehicleCommand.MAX_THROTTLE
                command.brake = 0
                command.reverse = False

            elif s:
                command.throttle = 0
                command.brake = VehicleCommand.MAX_BRAKE
                command.reverse = False

                if almost_stopped:
                    self.drive_mode = "REVERSE"

            else:
                command.throttle = 1
                command.brake = 0
                command.reverse = False

        else:  # REVERSE

            if s:
                command.throttle = VehicleCommand.MAX_THROTTLE
                command.brake = 0
                command.reverse = True

            elif w:
                command.throttle = 0
                command.brake = VehicleCommand.MAX_BRAKE
                command.reverse = True

                if almost_stopped:
                    self.drive_mode = "FORWARD"

            else:
                command.throttle = 1
                command.brake = 0
                command.reverse = True

        # ----------------------------------------
        # Steering
        # ----------------------------------------

        command.steering = VehicleCommand.CENTER_STEERING

        if keys[pygame.K_a]:
            command.steering = max(0, command.steering - 8)

        if keys[pygame.K_d]:
            command.steering = min(VehicleCommand.MAX_STEERING, command.steering + 8)