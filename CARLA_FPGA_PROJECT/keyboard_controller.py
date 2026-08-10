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

        self.headlight_auto = True
        self.hazard_auto = True
        self.manual_headlight_state = False
        self.manual_hazard_state = False

        # "FORWARD": W=가속, S=브레이크(정지 유지 시 후진 전환)
        # "REVERSE": S=후진가속, W=브레이크(정지 유지 시 전진 복귀)
        self.drive_mode = "FORWARD"
        self.steer_val = 12.0  # 부드러운 조향을 위한 내부 상태 저장 (중앙=12)

        self.camera_mode = "third_person"
        self.camera_yaw = 0.0
        self.camera_pitch = -10.0
        self.right_click_held = False

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

                if event.key == pygame.K_y:
                    self.hazard_auto = True
                if event.key == pygame.K_o:
                    self.headlight_auto = True
                if event.key == pygame.K_h:
                    self.hazard_auto = False
                    self.manual_hazard_state = not self.manual_hazard_state
                if event.key == pygame.K_l:
                    self.headlight_auto = False
                    self.manual_headlight_state = not self.manual_headlight_state

                if event.key == pygame.K_c:
                    self.camera_mode = "first_person" if self.camera_mode == "third_person" else "third_person"

            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 3:  # Right click
                    self.right_click_held = True

            if event.type == pygame.MOUSEBUTTONUP:
                if event.button == 3:
                    self.right_click_held = False

            if event.type == pygame.MOUSEMOTION:
                if self.right_click_held and self.camera_mode == "third_person":
                    # 마우스 x이동 -> yaw 회전, y이동 -> pitch 조절
                    dx, dy = event.rel
                    self.camera_yaw += dx * 0.5
                    self.camera_pitch = max(-80.0, min(20.0, self.camera_pitch - dy * 0.5))
                    
        return "run"

    # ======================================================
    # Driving Command (Manual Mode)
    # ======================================================

    def update(self, command, vehicle_speed_kmh):

        keys = pygame.key.get_pressed()

        w = keys[pygame.K_w]
        s = keys[pygame.K_s]
        a = keys[pygame.K_a]
        d = keys[pygame.K_d]

        # w,a,s,d 중 하나라도 누르면 즉시 수동 모드로 전환
        if w or a or s or d:
            self.manual_mode = True

        command.manual_mode = self.manual_mode
        command.autonomous_control = (not self.manual_mode)

        command.headlight_auto = self.headlight_auto
        command.hazard_auto = self.hazard_auto
        command.manual_headlight_state = self.manual_headlight_state
        command.manual_hazard_state = self.manual_hazard_state

        if not self.manual_mode:
            return

        almost_stopped = vehicle_speed_kmh < 2.0

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
        # Steering (Smooth)
        # ----------------------------------------

        steer_speed = 1.0  # 핸들을 꺾는 속도 감도 (작을수록 부드러움)
        return_speed = 1.0 # 핸들이 중앙으로 풀리는 속도 감도

        if keys[pygame.K_a]:
            self.steer_val = max(0.0, self.steer_val - steer_speed)
        elif keys[pygame.K_d]:
            self.steer_val = min(float(VehicleCommand.MAX_STEERING), self.steer_val + steer_speed)
        else:
            # Auto-center (키에서 손을 떼면 중앙 12로 서서히 복귀)
            if self.steer_val > 12.0:
                self.steer_val = max(12.0, self.steer_val - return_speed)
            elif self.steer_val < 12.0:
                self.steer_val = min(12.0, self.steer_val + return_speed)

        command.steering = int(self.steer_val)