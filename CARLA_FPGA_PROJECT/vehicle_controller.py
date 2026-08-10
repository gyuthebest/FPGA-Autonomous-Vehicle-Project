"""
==========================================================
CARLA FPGA Autonomous Driving Project

Vehicle Controller

VehicleCommand

↓

CARLA VehicleControl

==========================================================
"""

import carla
import math
import time

from vehicle_command import VehicleCommand


class VehicleController:

    # ======================================================

    MAX_THROTTLE = VehicleCommand.MAX_THROTTLE
    MAX_BRAKE = VehicleCommand.MAX_BRAKE

    MAX_STEERING = VehicleCommand.MAX_STEERING
    CENTER_STEERING = VehicleCommand.CENTER_STEERING

    MAX_GEAR = 3
    MIN_GEAR = 0
    
    SHIFT_DELAY = 0.5

    IDLE_RPM = 800
    IDLE_RPM = 800
    DOWNSHIFT_RPM = 3000
    UPSHIFT_RPM = 6500
    MAX_RPM = 7000

    # ======================================================
    # Fixed Route (교차로 진행방향 재계산으로 인한
    # 좌회전 시 지그재그/얼탐 방지 — 경로를 한 번만 결정하고 고정한다)
    # ======================================================

    ROUTE_STEP = 3.0
    ROUTE_MIN_REMAINING = 150.0
    ROUTE_EXTEND_LENGTH = 150.0
    ROUTE_SEARCH_LOOKBACK = 2
    ROUTE_SEARCH_WINDOW = 6
    ROUTE_RESET_THRESHOLD = 15.0

    # ======================================================
    # Continuous Steering Scaling (속도 구간 경계에서
    # 조향 계수가 계단식으로 뚝 바뀌며 회전 중 요동을
    # 유발하던 문제를 없애기 위해 전부 선형보간으로 처리)
    # -----------------------------
    # Tuning Parameters (Highway Curve Stable)
    # -----------------------------
    LOOKAHEAD_POINTS = [(0, 5.0), (20, 7.0), (50, 14.0), (80, 22.0), (100, 30.0), (120, 40.0), (150, 50.0)]
    STEER_VELOCITY_POINTS = [(0, 0.06), (25, 0.06), (60, 0.05), (100, 0.04), (120, 0.03), (150, 0.02)]
    STEER_GAIN_POINTS = [(0, 0.80), (30, 0.80), (60, 0.75), (100, 0.65), (120, 0.55), (150, 0.50)]
    STEER_RATE_SCALE_POINTS = [(0, 1.0), (30, 1.0), (50, 0.90), (70, 0.80), (90, 0.75), (120, 0.65), (150, 0.60)]

    # 코너링 시 편안하다고 느끼는 최대 횡가속도 (m/s^2)
    LATERAL_ACCEL_LIMIT = 3.0

    # ======================================================
    # Vehicle Specification (SEAT Leon)
    # ======================================================

    FINAL_DRIVE = 4.10

    GEAR_RATIO = [
        9.47,   # 1단 (0 ~ 20 km/h)
        3.44,   # 2단 (20 ~ 55 km/h)
        2.37,   # 3단 (55 ~ 80 km/h)
        1.65    # 4단 (80 ~ 120 km/h)
    ]

    TIRE_DIAMETER = 0.634      # m (195/65R15)

    # ======================================================

    def __init__(self, world, vehicle):

        self.vehicle = vehicle

        self.control = carla.VehicleControl()
        self.current_gear = 0

        self.current_rpm = self.IDLE_RPM

        self.last_shift_time = time.time()

        self.current_steering = 0.0  # STEP 10: 현재 조향각 상태 저장
        self.last_update_time = time.time() # STEP 1 적용: 시간 기반 조향 제한용

        self.target_steering = 0.0

        # 한 프레임 최대 조향 변화량
        self.max_steering_speed = 0.025
        self.max_steer_velocity = 0.012 
        self.vehicle_speed = 0.0
        self.curve_speed_limit = 999.0
        self.upcoming_turn_speed_limit = 999.0
        self.posture_speed_limit = 999
        self.locked_path_ids = []
        self.gear_down_timer = 0.0
        
        # 고정 주행 경로: (x, y, forward_x, forward_y, 누적거리)
        self.route = []
        self.route_index = 0

        self.world = world
        self.map = world.get_map()

        

    # ======================================================

    def update(self, command, posture_result=None):

        self.update_vehicle_state()

        is_reverse = command.manual_mode and getattr(command, "reverse", False)

        # --------------------------------------------------
        # Throttle & Brake
        # --------------------------------------------------

        self.control.throttle = self.convert_throttle(
            command.throttle
        )

        self.control.brake = self.convert_brake(
            command.brake
        )

        # -------------------------
        # Brake Override
        # -------------------------

        if self.control.brake > 0:
            self.control.throttle = 0.0

        self.apply_engine_brake(command)
        self.apply_creep(command)
        
        self.update_rpm(reverse=is_reverse)

        if posture_result is not None:

            self.posture_speed_limit = (

                posture_result.recommended_speed

            )

        else:

            self.posture_speed_limit = 999

        command.speed_limit = min(
            command.speed_limit,
            self.posture_speed_limit
        )

        self.apply_speed_limit(command)

        # --------------------------------------------------
        # Transmission
        # --------------------------------------------------
        if is_reverse:
            self.control.reverse = True
            self.control.manual_gear_shift = True
            self.control.gear = -1
            self.current_gear = 0
        else:
            self.control.reverse = False
            self.process_gear(command)

        # --------------------------------------------------
        # Steering
        # --------------------------------------------------
        if command.manual_mode:
            
            steer = (
                command.steering
                - VehicleCommand.CENTER_STEERING
            ) / float(VehicleCommand.CENTER_STEERING)
            
            steer = max(
                -1.0,
                min(
                    1.0,
                    steer
                )
            )
            
            self.current_steering += (
                steer - self.current_steering
            ) * 0.25
            self.last_update_time = time.time()
            
        else:
            
            steer = self.calculate_lane_steering()
            
            rate_scale = self.piecewise_linear(self.vehicle_speed, self.STEER_RATE_SCALE_POINTS)

            limit = (
                command.steering_rate_limit
                / 100.0
            ) * rate_scale
                
            steer *= limit
            
            current_time = time.time()
            
            dt = current_time - self.last_update_time
            
            self.last_update_time = current_time

            # 단일 rate limiter로만 부드럽게 이어지도록 한다.
            # (이전엔 rate limiter로 완만하게 제한해놓고 바로 이어서
            #  low-pass filter가 그 값을 다시 raw 목표쪽으로 크게
            #  당겨버려서, 두 메커니즘이 서로를 무력화시키며
            #  스냅→보정을 반복하는 요동의 원인이 되었다)
            
            max_delta = self.max_steer_velocity * dt * 60.0
            
            delta = steer - self.current_steering
            
            delta = max(
                -max_delta,
                min(max_delta, delta)
            )
            
            self.current_steering += delta
            
            self.current_steering = max(
                -1.0,
                min(
                    1.0,
                    self.current_steering
                )
            )
            
        self.control.steer = self.current_steering

        if command.emergency_stop:
            self.control.throttle = 0.0
            self.control.brake = 1.0
        
        self.vehicle.apply_control(self.control)

    # ======================================================

    def get_fpga_rpm_level(self):

        if self.current_rpm < 3000:
            return 0
        elif self.current_rpm < 5000:
            return 1
        elif self.current_rpm < 6500:
            return 2
        return 3

    # ======================================================

    def convert_throttle(self, throttle):
        """
        FPGA Throttle (0~10)
        →
        CARLA Throttle (0.0~1.0)
        """
        throttle = max(0, min(self.MAX_THROTTLE, throttle))
        return throttle / self.MAX_THROTTLE

    # ======================================================

    def convert_brake(self, brake):
        """
        FPGA Brake (0~10)
        →
        CARLA Brake (0.0~1.0)
        """
        brake = max(0, min(self.MAX_BRAKE, brake))
        return brake / self.MAX_BRAKE

    # ======================================================

    def process_gear(self, command):
        """
        Automatic Transmission
        """
        current_time = time.time()

        if (current_time - self.last_shift_time >= self.SHIFT_DELAY):
            
            if command.gear_down_request:
                if (self.current_rpm <= self.DOWNSHIFT_RPM and self.current_gear > self.MIN_GEAR):
                    self.current_gear -= 1
                    self.last_shift_time = current_time

            elif (
                self.current_rpm >= self.UPSHIFT_RPM
                and self.vehicle_speed > 15
                and command.brake == 0
            ):
                if self.current_gear < self.MAX_GEAR:
                    self.current_gear += 1
                    self.last_shift_time = current_time

        self.control.manual_gear_shift = True
        self.control.gear = self.current_gear + 1

    def piecewise_linear(self, x, points):
        """
        구간별 선형보간. 속도 경계(30/60km/h 등)를 지날 때
        조향 계수가 계단식으로 뚝 바뀌지 않고 부드럽게 이어지게 한다.
        """
        if x <= points[0][0]:
            return points[0][1]

        for i in range(len(points) - 1):
            x0, y0 = points[i]
            x1, y1 = points[i + 1]
            if x <= x1:
                t = (x - x0) / (x1 - x0)
                return y0 + t * (y1 - y0)

        return points[-1][1]

    def update_vehicle_state(self):
        velocity = self.vehicle.get_velocity()
        speed = math.sqrt(
            velocity.x**2 +
            velocity.y**2 +
            velocity.z**2
        )
        self.vehicle_speed = speed * 3.6 # km/h

    def update_rpm(self, reverse=False):
        """
        차량 속도와 현재 기어(후진 시 1단 기어비 고정)를 이용하여
        엔진 RPM 계산
        """
        wheel_circumference = math.pi * self.TIRE_DIAMETER
        wheel_rpm = (self.vehicle_speed * 1000 / 60) / wheel_circumference

        gear_ratio = self.GEAR_RATIO[0] if reverse else self.GEAR_RATIO[self.current_gear]

        engine_rpm = wheel_rpm * gear_ratio * self.FINAL_DRIVE

        self.current_rpm = int(
            max(self.IDLE_RPM, min(engine_rpm, self.MAX_RPM))
        )

    def apply_engine_brake(self, command):
        """
        엔진 브레이크
        Throttle = 0, Brake = 0 인 경우 자연 감속을 추가한다.
        기어가 낮을수록(다운시프트 상태) 기어비가 커서
        엔진브레이크가 더 강하게 걸리도록 기어비에 비례해 스케일한다.
        """
        if (command.throttle == 0 and command.brake == 0 and self.vehicle_speed > 5):

            gear_ratio = self.GEAR_RATIO[self.current_gear]
            top_gear_ratio = self.GEAR_RATIO[-1]

            engine_brake_factor = gear_ratio / top_gear_ratio

            brake_value = 0.06 * engine_brake_factor

            self.control.brake = max(self.control.brake, brake_value)

    def apply_creep(self, command):
        """
        Torque Converter Creep
        """
        if (command.throttle == 0 and command.brake == 0 and self.vehicle_speed < 5):
            self.control.throttle = max(self.control.throttle, 0.12)


    def apply_speed_limit(self, command):
        """
        제한속도 초과 시 자연스럽게 감속
        """

        if self.vehicle_speed <= command.speed_limit:
            self.gear_down_timer = 0.0
            return

        # 1단계 : 악셀 OFF (즉시 반응 - 커브/안전 대응 우선)
        self.control.throttle = 0.0

        # 2단계 : 기어다운 요청은 짧은 노이즈성 초과로 매번 걸리지 않도록
        # 0.2초 이상 지속 초과할 때만 요청한다 (와리가리 방지, 스로틀/브레이크와 분리)
        self.gear_down_timer += 1.0 / 30.0

        if self.current_rpm <= self.DOWNSHIFT_RPM and self.gear_down_timer >= 0.2:
            command.gear_down_request = True

        # 3단계 : 많이 초과하면 풋브레이크 (즉시 반응)
        speed_error = self.vehicle_speed - command.speed_limit

        if speed_error > 25:
            self.control.brake = max(self.control.brake, 0.7)
        elif speed_error > 15:
            self.control.brake = max(self.control.brake, 0.45)
        elif speed_error > 5:
            self.control.brake = max(self.control.brake, 0.2)

    def extend_route(self, distance_needed):
        """
        고정 경로를 distance_needed(m)만큼 뒤에 이어붙인다.
        분기점(교차로)에서 어느 방향으로 갈지는 여기서 한 번만
        결정되고, 이후 재계산되지 않고 route에 영구히 고정된다.
        """
        if not self.route:
            start_wp = self.map.get_waypoint(
                self.vehicle.get_location(),
                project_to_road=True
            )
            forward = self.vehicle.get_transform().get_forward_vector()
            cursor = start_wp
            cum = 0.0
            self.route.append((
                start_wp.transform.location.x,
                start_wp.transform.location.y,
                start_wp.transform.location.z,
                forward.x,
                forward.y,
                cum
            ))
        else:
            last_x, last_y, last_z, last_fx, last_fy, cum = self.route[-1]
            cursor = self.map.get_waypoint(
                carla.Location(last_x, last_y, last_z),
                project_to_road=True
            )
            forward = carla.Vector3D(last_fx, last_fy, 0.0)

        added = 0.0

        while added < distance_needed:

            options = cursor.next(self.ROUTE_STEP)

            if not options:
                break

            valid = [
                cand for cand in options
                if (
                    cand.transform.get_forward_vector().x * forward.x
                    + cand.transform.get_forward_vector().y * forward.y
                ) > 0.0
            ]

            if not valid:
                break

            nxt = max(
                valid,
                key=lambda cand: (
                    cand.transform.get_forward_vector().x * forward.x
                    + cand.transform.get_forward_vector().y * forward.y
                )
            )

            cum += self.ROUTE_STEP
            forward = nxt.transform.get_forward_vector()

            self.route.append((
                nxt.transform.location.x,
                nxt.transform.location.y,
                nxt.transform.location.z,
                forward.x,
                forward.y,
                cum
            ))

            cursor = nxt
            added += self.ROUTE_STEP

    def find_nearest_route_index(self, location):
        """
        route 배열에서 현재 위치와 가장 가까운 지점의 인덱스를 찾는다.
        직전 프레임 인덱스 근처만 탐색해서 매 프레임 전체 탐색을 피한다.
        고도(z) 차이도 포함해서, 다리 위/아래처럼 x,y는 비슷해도
        z가 크게 다른 지점을 같은 경로로 착각하지 않게 한다.
        """
        search_start = max(0, self.route_index - self.ROUTE_SEARCH_LOOKBACK)
        search_end = min(len(self.route), self.route_index + self.ROUTE_SEARCH_WINDOW)

        best_i = search_start
        best_d = float("inf")

        for i in range(search_start, search_end):
            x, y, z, _, _, _ = self.route[i]
            d = (x - location.x) ** 2 + (y - location.y) ** 2 + (z - location.z) ** 2

            if d < best_d:
                best_d = d
                best_i = i

        return best_i, math.sqrt(best_d)

    def estimate_route_curvature(self, start_index, distance):
        """
        route 위 start_index 지점으로부터 distance(m) 앞까지의
        heading 변화를 이용해 곡률(≈1/R)을 추정한다.
        """
        if start_index >= len(self.route):
            return 0.0

        start_cum = self.route[start_index][5]
        target_cum = start_cum + distance

        i = start_index

        while i < len(self.route) - 1 and self.route[i][5] < target_cum:
            i += 1

        fx0, fy0 = self.route[start_index][3], self.route[start_index][4]
        fx1, fy1 = self.route[i][3], self.route[i][4]

        yaw0 = math.degrees(math.atan2(fy0, fx0))
        yaw1 = math.degrees(math.atan2(fy1, fx1))

        diff = abs(yaw1 - yaw0)
        if diff > 180:
            diff = 360 - diff

        actual_distance = self.route[i][5] - start_cum

        if actual_distance < 0.1:
            return 0.0

        return math.radians(diff) / actual_distance

    def calculate_lane_steering(self):
        """
        Pure Pursuit 기반 차선 추종.
        고정 경로(self.route) 위에서 목표점을 찾으며,
        교차로 분기 결정은 경로 생성 시 한 번만 이루어진다
        (매 프레임 재계산으로 인한 좌회전 지그재그 방지).
        """

        self.max_steer_velocity = self.piecewise_linear(self.vehicle_speed, self.STEER_VELOCITY_POINTS)

        transform = self.vehicle.get_transform()
        location = transform.location

        base_lookahead = self.piecewise_linear(self.vehicle_speed, self.LOOKAHEAD_POINTS)

        if not self.route:
            self.extend_route(150.0)

        nearest_index, nearest_dist = self.find_nearest_route_index(location)

        if nearest_dist > self.ROUTE_RESET_THRESHOLD:
            # 경로에서 크게 벗어남 (수동 주행 등) -> 현재 위치 기준으로 재생성
            self.route = []
            self.route_index = 0
            self.extend_route(150.0)
            nearest_index, nearest_dist = self.find_nearest_route_index(location)

        self.route_index = nearest_index

        remaining = self.route[-1][5] - self.route[nearest_index][5]

        if remaining < self.ROUTE_MIN_REMAINING:
            self.extend_route(self.ROUTE_EXTEND_LENGTH)

        # ------------------------------------------
        # 급커브가 앞에 있으면 lookahead를 미리 줄인다.
        # (그대로 두면 목표점이 코너 저편의 직선 구간을 가리켜서
        # 회전을 늦게 시작하다가 반대 차선까지 넘어가는 원인이 된다)
        # ------------------------------------------

        upcoming_curve = self.estimate_route_curvature(nearest_index, base_lookahead)
        lookahead = base_lookahead / (1.0 + 10.0 * upcoming_curve)
        lookahead = max(3.0, lookahead)

        # ------------------------------------------
        # Lookahead 지점을 차량의 현재 위치(location)를 기준으로
        # 유클리드 거리를 사용해 완벽하게 연속적으로 찾는다.
        # (기존 누적거리 방식의 3m 계단식 점프 현상 원천 차단 -> 스무스한 조향)
        # ------------------------------------------
        target_i = nearest_index
        while target_i < len(self.route) - 1:
            cx, cy, _, _, _, _ = self.route[target_i]
            dist = math.sqrt((cx - location.x)**2 + (cy - location.y)**2)
            if dist >= lookahead:
                break
            target_i += 1

        if target_i > nearest_index:
            px, py, _, _, _, _ = self.route[target_i - 1]
            cx, cy, _, _, _, _ = self.route[target_i]
            
            d1 = math.sqrt((px - location.x)**2 + (py - location.y)**2)
            d2 = math.sqrt((cx - location.x)**2 + (cy - location.y)**2)
            
            if d2 - d1 > 0.001:
                t = (lookahead - d1) / (d2 - d1)
                t = max(0.0, min(1.0, t))
            else:
                t = 0.0
            
            target_x = px + t * (cx - px)
            target_y = py + t * (cy - py)
        else:
            target_x, target_y, _, _, _, _ = self.route[target_i]

        vehicle_yaw = math.radians(transform.rotation.yaw)

        dx = target_x - location.x
        dy = target_y - location.y

        local_y = (
            -math.sin(vehicle_yaw) * dx
            + math.cos(vehicle_yaw) * dy
        )

        if lookahead < 0.1:
            return 0.0

        curvature = (
            2.0 * local_y
            / (lookahead * lookahead)
        )

        # ------------------------------------------
        # Curve Speed Planner (물리 기반 연속 공식)
        # v_limit = sqrt(a_lateral_max / curvature)
        # ------------------------------------------

        curve_now = abs(curvature)

        speed_ms = self.vehicle_speed / 3.6
        preview_distance = max(30.0, speed_ms * 3.0)

        curve_preview = self.estimate_route_curvature(nearest_index, preview_distance)

        curve_value = max(curve_now, curve_preview)

        # 고속도로의 완만한 커브에서도 사전에 부드럽게 감속하기 위해 임계값을 0.001로 하향
        if curve_value > 0.001:
            target_turn_limit = math.sqrt(self.LATERAL_ACCEL_LIMIT / curve_value) * 3.6
            target_turn_limit = min(target_turn_limit, 999.0)
        else:
            target_turn_limit = 999.0

        if target_turn_limit < self.upcoming_turn_speed_limit:
            self.upcoming_turn_speed_limit = target_turn_limit
        else:
            self.upcoming_turn_speed_limit = min(
                target_turn_limit,
                self.upcoming_turn_speed_limit + 5.0
            )

        # 포드 머스탱 (vehicle.ford.mustang) 스펙에 맞춘 축간거리 (2.72m)
        wheelbase = 2.72

        steer = math.atan(
            wheelbase * curvature
        )

        steer /= math.radians(35)

        steer_gain = self.piecewise_linear(self.vehicle_speed, self.STEER_GAIN_POINTS)
        steer *= steer_gain

        steer = max(-1.0, min(1.0, steer))

        if abs(steer) < 0.008:
            steer = 0.0

        return steer