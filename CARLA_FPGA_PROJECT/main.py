import random
import carla
import pygame
import numpy as np
import gc

import utils
from sensor_manager import SensorManager
from keyboard_controller import KeyboardController
from vehicle_controller import VehicleController
from vehicle_command import VehicleCommand
from camera_manager import CameraManager
from environment_manager import EnvironmentManager
from perception_manager import PerceptionManager
from scenario_manager import ScenarioManager
from map_manager import MapManager
from obstacle_manager import ObstacleManager
from weather_manager import WeatherManager
from environment import Environment
from csv_logger import CSVLogger

from ttc_logic import TTCLogic
from posture_logic import PostureLogic
from road_logic import RoadLogic
from vision_logic import VisionLogic
from fusion_logic import FusionLogic
from fpga_interface import FPGAInterface, build_input_words
from pl_verification_logger import PLVerificationLogger
from sensor_noise import SensorNoiseModel
from control_panel import ControlPanel
from world_scenario_controller import WorldScenarioController
from dashboard import draw_dashboard
from live_scenario_verifier import LiveScenarioVerifier

import os

HOST = "127.0.0.1"
PORT = 2000
VEHICLE_ID = "vehicle.ford.mustang"

WEATHER_CYCLE_INTERVAL = 30.0
WINDOWED_SIZE = (1280, 720)
ENABLE_LEGACY_AUTO_RAMPS = False
ENABLE_LEGACY_AUTO_WEATHER = False
ENABLE_LEGACY_AUTO_OBSTACLES = False


def env_flag(name, default=True):
    """Read a conventional boolean environment variable."""
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip().lower() not in ("0", "false", "no", "off")


# --- situation (3비트) : CARLA가 PL로 알려주는 주행 상황 ---------------------
# PL은 이 값으로 jump/consistency 마스크를 결정하므로 인코딩이 정확해야 한다.
#   000 정지        : consistency_mask_4/5/6 을 열어 "정지 시 각속도 ~ 0" 검사 활성화
#   001 장애물 등장  : distance/approach_speed jump 마스크
#   010 자세변화    : distance/approach_speed jump 마스크
#   011 날씨 변화   : 온도/습도/조도 jump 마스크
#   100 정상 주행   : 마스크 없음
SITUATION_STOPPED = 0
SITUATION_OBSTACLE = 1
SITUATION_POSTURE = 2
SITUATION_WEATHER = 3
SITUATION_NORMAL = 4

# 자세변화 판정 각속도 한계 (rad/s). 센서 높이 1.5 m 가정.
POSTURE_RATE_LIMIT_RPS = 0.340
# 정지 판정 속도 (1 km/h)
STOPPED_SPEED_MPS = 0.278
# 레이더 무표적 sentinel (m)
OBSTACLE_SENTINEL_M = 200.0


def classify_situation(sensor, prev_distance, environment_changing):
    """주행 상황을 3비트 situation 코드로 분류한다.

    우선순위: 날씨변화 > 자세변화 > 장애물등장 > 정지 > 정상.
    사건(001/010/011)이 상태(000/100)보다 우선한다. 사건 구간에서 PL의
    jump/consistency 마스크가 열려야 정상적인 값 계단이 고장으로 오판되지
    않기 때문이다.
    """
    if environment_changing:
        return SITUATION_WEATHER
    if (abs(sensor.gyro_x) > POSTURE_RATE_LIMIT_RPS
            or abs(sensor.gyro_y) > POSTURE_RATE_LIMIT_RPS
            or abs(sensor.gyro_z) > POSTURE_RATE_LIMIT_RPS):
        return SITUATION_POSTURE
    if (prev_distance >= OBSTACLE_SENTINEL_M
            and float(sensor.distance) < OBSTACLE_SENTINEL_M):
        return SITUATION_OBSTACLE
    if abs(float(sensor.speed_x)) <= STOPPED_SPEED_MPS:
        return SITUATION_STOPPED
    return SITUATION_NORMAL


def should_apply_fpga_output(control_panel, fpga_result):
    """Preserve PL authority for a latched TD/MRM after a test input clears."""
    return bool(
        fpga_result is not None
        and control_panel.apply_fpga_output
        and (
            control_panel.intervention_scenario_active
            or fpga_result.transition_demand
            or fpga_result.mrm
        )
    )


def create_main_display(fullscreen=None):
    """Create a resizable window or a fullscreen display.

    The normal window is the default so the Windows maximize button works.
    Set CARLA_FULLSCREEN=1 to start fullscreen, or press F11 while running.
    """
    if fullscreen is None:
        fullscreen = env_flag("CARLA_FULLSCREEN", default=False)

    flags = pygame.DOUBLEBUF
    if fullscreen:
        screen = pygame.display.set_mode((0, 0), flags | pygame.FULLSCREEN)
        print(f"[Display] Fullscreen: {screen.get_width()}x{screen.get_height()}")
    else:
        screen = pygame.display.set_mode(WINDOWED_SIZE, flags | pygame.RESIZABLE)
        print(f"[Display] Windowed: {screen.get_width()}x{screen.get_height()}")
    return screen


def toggle_main_display(screen):
    """Toggle between a resizable window and fullscreen mode."""
    is_fullscreen = bool(screen.get_flags() & pygame.FULLSCREEN)
    return create_main_display(fullscreen=not is_fullscreen)


def create_hud_font(screen):
    font_size = max(13, min(16, screen.get_height() // 60))
    return pygame.font.SysFont("consolas", font_size)

ENGINE_SOUND_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets", "sounds", "engine_loop.wav")

# 음원 파일을 녹음/제작할 때 기준이 된 RPM
# (이 RPM에서 재생하면 피치 변화 없이 원본 그대로 들린다)
ENGINE_REFERENCE_RPM = 800


def load_engine_base_array():
    """
    엔진 루프 음원을 로드해 float 배열로 반환한다.
    (리샘플링 연산은 float 상태에서 하고, Sound 생성 시점에 int16으로 변환 )
    """
    base_sound = pygame.mixer.Sound(ENGINE_SOUND_PATH)
    array = pygame.sndarray.array(base_sound)
    return array.astype(np.float64)


def resample_pitch(base_array, ratio):
    """
    base_array를 ratio배 빠르게(=피치를 높게) 재생한 것처럼 리샘플링한다.
    ratio > 1: RPM이 기준보다 높음 (피치업)
    ratio < 1: RPM이 기준보다 낮음 (피치다운)
    """
    n = base_array.shape[0]
    new_n = max(2, int(n / ratio))

    old_idx = np.arange(n)
    new_idx = np.linspace(0, n - 1, new_n)

    if base_array.ndim == 1:
        resampled = np.interp(new_idx, old_idx, base_array)
    else:
        channels = base_array.shape[1]
        resampled = np.zeros((new_n, channels))
        for c in range(channels):
            resampled[:, c] = np.interp(new_idx, old_idx, base_array[:, c])

    resampled = np.clip(resampled, -32768, 32767).astype(np.int16)
    
    # 1D 모노 사운드일 경우, 2채널(스테레오) 믹서에 맞게 2D(Stereo)로 복제해야 함
    if resampled.ndim == 1:
        resampled = np.column_stack((resampled, resampled))
        
    return pygame.sndarray.make_sound(np.ascontiguousarray(resampled))

def run_session(world, map_manager, screen, font, clock, keyboard, traffic_manager,
                engine_pitch_cache_global, control_panel):
    """
    차량 스폰부터 세션 종료까지 한 번의 주행을 실행한다.
    반환값: 'restart' | 'quit'
    """

    # set_mode() replaces the display surface when F11 was used in an earlier
    # session. Always start from pygame's current display surface.
    screen = pygame.display.get_surface() or screen
    font = create_hud_font(screen)

    vehicle = None
    camera = None
    obstacle_manager = None
    logger = None
    pl_verify_logger = None
    fpga = None
    world_scenarios = None
    ramp_actors = []
    
    gc.disable()

    try:
        bp_lib = world.get_blueprint_library()
        try:
            vehicle_bp = bp_lib.find(VEHICLE_ID)
        except:
            vehicle_bp = bp_lib.filter("vehicle.*")[0]

        if vehicle_bp.has_attribute('color'):
            vehicle_bp.set_attribute('color', '130,0,10') # 크림슨(진한 붉은색) 적용

        spawn = random.choice(world.get_map().get_spawn_points())

        vehicle = world.spawn_actor(vehicle_bp, spawn)
        vehicle.set_autopilot(False)

        # --- [ROLL TEST] 차량 서스펜션 세팅 (조금 더 부드럽게) ---
        physics_control = vehicle.get_physics_control()
        physics_control.suspension_stiffness = 0.5  # 더 부드럽게 (적당한 롤링 허용)
        physics_control.suspension_damping_rate = 0.2
        vehicle.apply_physics_control(physics_control)

        camera = CameraManager(
            world,
            vehicle,
            image_width=screen.get_width(),
            image_height=screen.get_height(),
        )
        sensor = SensorManager(vehicle)

        environment_manager = EnvironmentManager(map_manager)
        obstacle_manager = ObstacleManager(world, vehicle, map_manager, traffic_manager)
        perception_manager = PerceptionManager(world, vehicle)
        controller = VehicleController(world, vehicle)
        scenario = ScenarioManager()
        weather_manager = WeatherManager(world)
        world_scenarios = WorldScenarioController(
            world, vehicle, weather_manager, control_panel,
        )
        live_verifier = LiveScenarioVerifier(control_panel, world_scenarios)
        sensor_noise = SensorNoiseModel()
        print(f"[SENSOR NOISE] {'enabled' if sensor_noise.enabled else 'disabled'}"
              " (CARLA_SENSOR_NOISE)")
        logger = CSVLogger()
        pl_verify_logger = PLVerificationLogger.from_environment()
        if pl_verify_logger.enabled:
            print(f"[PL VERIFY] Capture: {pl_verify_logger.capture_path}")
            print(f"[PL VERIFY] AXI vectors: {pl_verify_logger.vector_path}")

        try:
            fpga = FPGAInterface.from_environment()
            if fpga.enabled:
                print(f"[FPGA] UDP bridge enabled: {fpga.board_address[0]}:{fpga.board_address[1]}")
            else:
                print("[FPGA] Disabled by FPGA_ENABLED=0; software command fallback is active.")
        except OSError as exc:
            print(f"[FPGA] UDP initialization failed ({exc}); software command fallback is active.")
            fpga = FPGAInterface(enabled=False)


        
        try:
            if not pygame.mixer.get_init():
                pygame.mixer.init()

            mixer_info = pygame.mixer.get_init()
            print(f"[Audio] Mixer initialized: freq={mixer_info[0]}, format={mixer_info[1]}, channels={mixer_info[2]}")
            print(f"[Audio] Loading sound from: {ENGINE_SOUND_PATH}")

            if not os.path.isfile(ENGINE_SOUND_PATH):
                raise FileNotFoundError(f"엔진 사운드 파일을 찾을 수 없습니다: {ENGINE_SOUND_PATH}")

            engine_channel = pygame.mixer.Channel(0)
            last_engine_ratio_key = 1.0
            if 1.0 in engine_pitch_cache_global:
                engine_channel.play(engine_pitch_cache_global[1.0], loops=-1)
            print("[Audio] Engine sound started successfully.")

        except Exception as e:
            import traceback
            print("=" * 60)
            print("[ERROR] Audio mixer failed to initialize. Running without sound.")
            traceback.print_exc()
            print("=" * 60)
            engine_channel = None

        keyboard.manual_mode = False
        keyboard.drive_mode = "FORWARD"

        simulation_time = 0.0
        sample_seq = 0  # Added for FPGA testing
        # timeout 주입 시 몇 프레임마다 한 표본을 통과시킬지.
        # 25 프레임 = 1.25초 침묵이면 TIMEOUT_N=10 (1초) 을 넘겨 확정된다.
        TIMEOUT_RELEASE_PERIOD = 25
        timeout_hold_frames = 0
        prev_mrm = False
        # PL transition-demand/MRM registers survive a Python-only restart.
        # Briefly assert the PL-facing MANUAL bit so stale safety state from a
        # previous scenario cannot contaminate a new validation run.
        pl_startup_reset_samples = 20

        # [추가된 부분: Jitter 측정용 변수 및 초시계 초기화]
        import time
        last_time = time.perf_counter()
        gap_max_ms = 0.0

        weather_cycle = [
            weather_manager.set_clear,
            weather_manager.set_fog,
            weather_manager.set_rain,
            weather_manager.set_snow,
        ]
        weather_index = 0
        weather_timer = 0.0
        weather_cycle[weather_index]()

        ttc_logic = TTCLogic()
        posture_logic = PostureLogic()
        road_logic = RoadLogic()
        vision_logic = VisionLogic()
        fusion_logic = FusionLogic()

        # [FPGA Situation Logic State Variables]
        prev_distance = 250.0
        prev_weather = -1
        prev_environment_signature = None
        environment_transition_samples = 0
        
        last_ramp_time = -20.0  # 시작하자마자 바로 한 번 생성되도록

        print("Vehicle Spawned.")
        print()
        print("========== CONTROL ==========")
        print("M   : AUTO / MANUAL")
        print("W   : Throttle / Reverse-Brake")
        print("S   : Brake / Reverse-Throttle")
        print("A/D : Steering")
        print("R   : Restart")
        print("F11 : Window / Fullscreen")
        print("ESC : Exit")
        print("=============================")
        # --- 무인(스크립트) 실행 제어 ---------------------------------
        # 검증을 재현 가능하게 돌리기 위한 시험용 스위치다. 주행 알고리즘에는
        # 영향을 주지 않는다.
        #   CARLA_APPLY_FPGA=0   : PL 출력을 차량에 적용하지 않는다(개루프 캡처)
        #   CARLA_RUN_SECONDS=N  : N초 뒤 정상 종료한다(로그를 온전히 닫는다)
        if not env_flag("CARLA_APPLY_FPGA", default=True):
            control_panel.apply_fpga_output = False
            print("[CONTROL] Apply FPGA output: OFF (CARLA_APPLY_FPGA=0)")
        # 무인 검증용 고장 사전 주입.  GUI 버튼을 누르지 않고도 같은 경로를
        # 탄다(control_panel.injector 를 그대로 쓴다).
        #   CARLA_INJECT_FAULT=distance:stuck,gyro_x:range
        #   CARLA_INJECT_RISK=collision
        for spec in (os.getenv("CARLA_INJECT_FAULT", "") or "").split(","):
            spec = spec.strip()
            if not spec:
                continue
            name, _, check = spec.partition(":")
            if name and check:
                control_panel.injector.toggle_sensor_fault(name, check)
                print(f"[INJECT] sensor fault {name}:{check}")
        for spec in (os.getenv("CARLA_INJECT_RISK", "") or "").split(","):
            spec = spec.strip()
            if spec:
                control_panel.injector.toggle_risk(spec)
                print(f"[INJECT] risk {spec}")

        run_seconds = float(os.getenv("CARLA_RUN_SECONDS", "0") or 0)
        run_deadline = (time.perf_counter() + run_seconds) if run_seconds > 0 else None
        if run_deadline is not None:
            print(f"[RUN] Auto-exit after {run_seconds:.0f} s")

        print()

        while True:

            if run_deadline is not None and time.perf_counter() >= run_deadline:
                print("[RUN] Duration reached, exiting cleanly.")
                return "quit"

            status = keyboard.poll_system_events(control_panel)

            if status in ("quit", "restart"):
                return status
            if status == "toggle_fullscreen":
                screen = toggle_main_display(screen)
                font = create_hud_font(screen)
                continue

            # Pygame 2 updates the display surface after a RESIZABLE window is
            # maximized or dragged. Use the new surface for this frame.
            screen = pygame.display.get_surface() or screen

            # --- [ROLL TEST] 20초마다 정면 우측에 경사로 스폰 ---
            if ENABLE_LEGACY_AUTO_RAMPS and simulation_time - last_ramp_time >= 20.0:
                last_ramp_time = simulation_time
                try:
                    ramp_bp = bp_lib.filter('static.prop.container')[0]
                    carla_map = world.get_map()
                    ego_loc = vehicle.get_location()
                    spawn_wp = carla_map.get_waypoint(ego_loc)
                    # 현재 차가 달리는 차선 기준 40m 앞
                    next_wps = spawn_wp.next(40.0)
                    
                    if next_wps:
                        target_wp = next_wps[0]
                        ramp_transform = target_wp.transform
                        right_vec = ramp_transform.get_right_vector()
                        
                        # 차로폭 절반(우측)만 덮도록 우측으로 1.2m 시프트
                        ramp_transform.location.x += right_vec.x * 1.2
                        ramp_transform.location.y += right_vec.y * 1.2
                        
                        # 컨테이너를 아주 깊게 바닥에 묻어서 윗면만 15~20cm 튀어나오게 만듦
                        # (거대한 벽이 아니라 길고 평평한 단상이 되어 우측 바퀴가 쉽게 올라탐)
                        ramp_transform.location.z -= 1.05
                        ramp_transform.rotation.pitch = 0.0
                        ramp_transform.rotation.roll = 0.0
                        
                        # 바닥 충돌로 인한 스폰 실패를 막기 위해 공중에 스폰 후 강제 이동
                        safe_transform = carla.Transform(ramp_transform.location, ramp_transform.rotation)
                        safe_transform.location.z += 20.0
                        ramp = world.spawn_actor(ramp_bp, safe_transform)
                        ramp.set_transform(ramp_transform)
                        
                        ramp_actors.append(ramp)
                except Exception as e:
                    print(f"[Warning] Dynamic Ramp spawn failed: {e}")

            # Apply panel-selected world/vehicle conditions before advancing
            # CARLA, so the sensors from this tick observe the new scenario.
            world_scenarios.update(1.0 / 20.0)
            carla_frame = world.tick()

            sensor.update()
            sensor.rpm = utils.rpm_to_level(controller.current_rpm)
            sensor.weather = weather_manager.weather_code
            sensor.temperature = weather_manager.temperature
            sensor.humidity = weather_manager.humidity

            perception = perception_manager.update()

            # Associate raw radar reflections with the route-level tracked
            # actor. CARLA radar points carry no actor ID; choosing the nearest
            # reflection alone mistakes road furniture for a lead vehicle.
            # The matched track supplies the coherent base measurement, after
            # which the fault injector produces the deliberately corrupted
            # sensor value sent to the PL.
            if perception.front_actor is None or perception.front_distance > 200.0:
                sensor.distance = 200.0
                sensor.approach_speed = 0.0
            else:
                sensor.distance = min(200.0, float(perception.front_distance))
                sensor.approach_speed = max(0.0, float(perception.relative_speed))

            # Convert visible CARLA test conditions into sustained sensor
            # responses before optional sensor-fault corruption.  This keeps
            # the physical scenario as the baseline and the fault injector as
            # the measurement-side failure, matching the PL data model.
            world_scenarios.apply_sensor_conditions(sensor)

            # Apply selected test faults before all decision logic and before
            # the exact same values are packed into the AXI register image.
            control_panel.injector.apply(sensor)

            # 기본 측정 잡음.  CARLA IMU는 잡음 stddev가 0이고 온습도는 상수,
            # 조도는 30프레임마다만 갱신되므로 delta == 0이 15표본 이상 이어져
            # 정상 주행에서도 PL stuck이 확정된다.  실제 센서라면 최하위
            # 비트가 항상 흔들리므로 그 특성을 되돌려 준다.
            #
            # 고장 주입 '뒤'에 두고 고장 채널만 건너뛴다.  위험도 주입이
            # 온도/습도/조도를 상수로 덮어쓰기 때문에, 앞에서 호출하면 노면·
            # 시야 위험도 시험이 매번 온습도 stuck까지 만들어 시험을 오염시킨다.
            sensor_noise.apply(
                sensor,
                skip=(control_panel.injector.frozen_channels
                      | world_scenarios.driven_channels),
            )
            environment = environment_manager.update(sensor, controller.upcoming_turn_speed_limit)

            # [FPGA Data Prep]
            control = vehicle.get_control()
            manual_command = VehicleCommand()
            keyboard.update(manual_command, sensor.speed)

            # ========================================================
            # [FPGA AXI REGISTER PACKING - TEST]
            # ========================================================
            # timeout 고장: sample_seq 를 대부분의 프레임에서 고정한다.
            #
            # PL 의 valid_s0 = (sample_seq_in != sample_seq_out) 이므로 seq 를
            # 고정하면 "새 표본이 없다"고 판단해 100 ms 마다 timeout 증거를
            # 쌓고 TIMEOUT_N=10 에서 확정한다(약 1초).
            #
            # 다만 **완전히 고정하면 화면에 안 보인다.**  risk_control.sv 는
            #     if (valid_in_rel) rel_out <= rel_in;
            # 이라 신뢰도 워드를 valid 표본에서만 래치한다.  seq 가 멈춰 있는
            # 동안에는 워드가 갱신되지 않아 마지막 값(NORMAL)이 그대로 남는다.
            # PL 은 timeout 을 확정하고 있는데 내보내지 않는 것이다.
            #
            # 그래서 주기적으로 한 표본만 통과시킨다.  RTL 의
            # timeout_confirm_hold(DROP_N=2) 가 확정된 timeout 을 복구 첫
            # 표본까지 유지하도록 만들어져 있어, 그 표본에서 11채널 INVALID 가
            # 래치된다.  이후 다시 고정하면 증거가 재축적되므로 화면은 계속
            # INVALID 를 유지한다.
            if control_panel.injector.drop_sample:
                timeout_hold_frames += 1
                if timeout_hold_frames >= TIMEOUT_RELEASE_PERIOD:
                    timeout_hold_frames = 0
                    sample_seq += 1
            else:
                timeout_hold_frames = 0
                sample_seq += 1
            
            # [추가된 부분: 파이썬 기반 루프 소요 시간 측정 (Hardware ILA 대체)]
            current_time = time.perf_counter()
            gap_ms = (current_time - last_time) * 1000.0
            last_time = current_time

            # 첫 번째 프레임(초기화 딜레이)은 무시하고, 역대 최장 시간이 갱신되면 출력
            if sample_seq > 1 and gap_ms > gap_max_ms:
                gap_max_ms = gap_ms
                print(f"[*] 최대 지터 갱신! 새로운 gap_max: {gap_max_ms:.2f} ms (프레임 seq: {sample_seq})")

            # --- Situation Logic (3-bit) ---
            # 000: Stopped, 001: Obstacle, 010: Posture, 011: Weather, 100: Normal
            current_weather = environment.weather
            current_distance = sensor.distance
            environment_signature = (
                current_weather,
                control_panel.road_surface,
                int(control_panel.visibility_risk),
            )
            if (prev_environment_signature is not None and
                    environment_signature != prev_environment_signature):
                # Temperature/humidity/lux legitimately step when the driver
                # selects an environmental scenario.  Hold situation=011 for
                # half a second so the PL jump predictor sees the complete
                # transition as an intentional environment change.
                environment_transition_samples = 10

            environment_changing = environment_transition_samples > 0
            if environment_changing:
                environment_transition_samples -= 1
            situation_val = classify_situation(
                sensor, prev_distance, environment_changing
            )


            prev_weather = current_weather
            prev_distance = current_distance
            prev_environment_signature = environment_signature
            
#             scenario.update(1.0 / 30.0, sensor)
# 
#             # -------------------------------
#             # Weather Cycle (30초마다 순환)
#             # -------------------------------
# 
#             weather_timer += 1.0 / 30.0
# 
#             if weather_timer >= WEATHER_CYCLE_INTERVAL:
#                 weather_timer = 0.0
#                 weather_index = (weather_index + 1) % len(weather_cycle)
#                 weather_cycle[weather_index]()
# 
#             simulation_time += 1.0 / 30.0

            scenario.update(1.0 / 20.0, sensor)

            # -------------------------------
            # Weather Cycle (20Hz 업데이트에 맞춰 순환)
            # -------------------------------

            weather_timer += 1.0 / 20.0

            if ENABLE_LEGACY_AUTO_WEATHER and weather_timer >= WEATHER_CYCLE_INTERVAL:
                weather_timer = 0.0
                weather_index = (weather_index + 1) % len(weather_cycle)
                weather_cycle[weather_index]()

            simulation_time += 1.0 / 20.0
            
            # -------------------------------
            # 신호등 강제 해킹 (접근 시 무조건 초록불)
            # -------------------------------
            if vehicle.is_at_traffic_light():
                traffic_light = vehicle.get_traffic_light()
                if traffic_light and traffic_light.get_state() != carla.TrafficLightState.Green:
                    traffic_light.set_state(carla.TrafficLightState.Green)
                    
            camera.update_transform(keyboard.camera_mode, keyboard.camera_yaw, keyboard.camera_pitch)

            screen.fill((0, 0, 0))

            if camera.image is not None:
                frame = camera.image
                frame = frame[:, :, ::-1]
                surface = pygame.surfarray.make_surface(frame.swapaxes(0, 1))
                if surface.get_size() != screen.get_size():
                    surface = pygame.transform.smoothscale(surface, screen.get_size())
                screen.blit(surface, (0, 0))

            # (Moved to top for FPGA)
            # (Moved to top for FPGA)

            ttc_result = ttc_logic.update(sensor, perception)
            road_result = road_logic.update(environment, sensor)
            vision_result = vision_logic.update(environment, sensor)
            posture_result = posture_logic.update(sensor)

            if keyboard.manual_mode:
                command = manual_command
            elif not control_panel.fpga_in_loop:
                # FPGA: NONE -- PL 뿐 아니라 **Python 위험 로직도 빼고**
                # 순수 기본 주행만 남긴다.  FPGA 유무를 비교하려면 기준선에
                # 같은 위험 대응이 들어 있으면 안 되기 때문이다.
                #
                # fuse() 는 인자가 None 이면 그 요소를 반영하지 않도록 이미
                # 되어 있다.  네 개를 모두 빼면 기본값만 남는다.
                #   throttle = MAX_THROTTLE, brake = 0,
                #   steering_rate_limit = 100, speed_limit = 999
                #
                # 조향은 자율주행 모드에서 command.steering 을 쓰지 않고
                # controller.calculate_lane_steering() 이 만들므로 차선 추종이
                # 그대로 유지된다.  steering_rate_limit 이 100 이라 위험
                # 로직에 의한 조향 제한만 사라진다.
                #
                # 다만 속도는 손을 대야 한다.  도로 제한속도는 road_logic 을
                # 거쳐서만 command.speed_limit 에 실리는데 그 로직을 뺐으므로,
                # environment 의 값을 직접 넣어 준다.  이렇게 해야
                # "위험도와 무관하게 도로 제한속도만 따른다" 가 된다.
                command = fusion_logic.fuse()
                command.speed_limit = float(environment.speed_limit)
            else:
                command = fusion_logic.fuse(
                    ttc=ttc_result,
                    posture=posture_result,
                    road=road_result,
                    vision=vision_result
                )

            command.manual_mode = keyboard.manual_mode
            command.autonomous_control = (not keyboard.manual_mode)
            
            # 자율주행 모드일 경우 fusion_logic이 새 객체를 만들며 조명 상태가 날아가므로, 키보드 컨트롤러의 상태를 다시 복사해줌
            command.headlight_auto = keyboard.headlight_auto
            command.hazard_auto = keyboard.hazard_auto
            command.manual_headlight_state = keyboard.manual_headlight_state
            command.manual_hazard_state = keyboard.manual_hazard_state

            # --------------------------------------------------------
            # CARLA laptop -> Zynq PS (UDP) -> PL pipeline -> PS -> CARLA
            # --------------------------------------------------------
            fpga_transition_demand = False
            fpga_hud_warning = False
            fpga_mrm = False
            fpga_td_remain_sec = 11
            fpga_headlight_auto_out = command.headlight
            fpga_hazard_auto_out = command.hazard

            if command.manual_mode:
                desired_steering = (
                    command.steering - VehicleCommand.CENTER_STEERING
                ) / float(VehicleCommand.CENTER_STEERING)
            else:
                desired_steering = controller.calculate_lane_steering()
                desired_steering *= command.steering_rate_limit / 100.0

            requested_speed_limit = min(
                float(environment.speed_limit),
                float(command.speed_limit),
            )
            fpga_manual_mode = bool(
                command.manual_mode
                or sample_seq < pl_startup_reset_samples
                or live_verifier.force_manual_mode
            )
            input_words = build_input_words(
                sample_seq=sample_seq,
                accel_xyz=(sensor.accel_x, sensor.accel_y, sensor.accel_z),
                gyro_xyz=(sensor.gyro_x, sensor.gyro_y, sensor.gyro_z),
                incline_xyz=(sensor.incline_x, sensor.incline_y, sensor.incline_z),
                speed_xyz=(sensor.speed_x, sensor.speed_y, sensor.speed_z),
                distance_m=sensor.distance,
                approach_speed_mps=sensor.approach_speed,
                temperature=sensor.temperature,
                humidity_pct=sensor.humidity,
                lux=sensor.lux,
                speed_limit_kmh=requested_speed_limit,
                weather=environment.weather,
                rpm_level=utils.rpm_to_level(controller.current_rpm),
                accelerator=command.throttle,
                brake=command.brake,
                steering_normalized=desired_steering,
                manual_mode=fpga_manual_mode,
                gear=0 if control.reverse else controller.current_gear + 1,
                headlight=command.headlight,
                hazard=command.hazard,
                situation=situation_val,
            )
            # Preserve the exact pre-control values that were packed for this
            # PL transaction.  The cockpit input monitor must not show the
            # post-FPGA command in place of the values that the FPGA received.
            fpga_input_snapshot = {
                "sample_seq": sample_seq,
                "speed_limit": requested_speed_limit,
                "rpm_level": utils.rpm_to_level(controller.current_rpm),
                "accelerator": int(command.throttle),
                "brake": int(command.brake),
                "steering": float(desired_steering),
                "manual_mode": bool(fpga_manual_mode),
                "gear": 0 if control.reverse else controller.current_gear + 1,
                "headlight": bool(command.headlight),
                "hazard": bool(command.hazard),
                "situation": int(situation_val),
            }

            host_send_ns = time.perf_counter_ns()
            # sample_seq 고정만으로 timeout 을 만들므로 송신은 계속한다.
            # 그래야 PL 이 확정한 INVALID 를 읽어서 화면에 띄울 수 있다.
            sample_dropped = control_panel.injector.drop_sample
            # FPGA: NONE 은 PL 을 루프에서 완전히 뺀다.  프레임을 보내지
            # 않으므로 UDP 왕복 지연과 지터가 주행 루프에 영향을 주지 않고,
            # 순수 CARLA 기본 주행이 기준선이 된다.  고장 주입과 위험도
            # 시나리오는 그대로 센서에 반영되므로 같은 조건에서 FPGA 유무만
            # 바꿔 비교할 수 있다.
            fpga_result = (
                fpga.exchange(input_words, sample_seq)
                if fpga is not None and control_panel.fpga_in_loop
                else None
            )
            control_panel.set_fpga_result(fpga_result)
            # FPGA is armed continuously, but it receives authority only while
            # a deliberate sensor-fault/risk demonstration is active.  Normal
            # frames keep the CARLA autonomous command generated above.
            # TD/MRM are latched PL safety states.  They retain control
            # authority after the initiating test button is released, until
            # manual takeover/reset clears the PL state.
            fpga_actuation_active = should_apply_fpga_output(control_panel, fpga_result)
            host_response_ns = time.perf_counter_ns()
            if pl_verify_logger is not None and not sample_dropped:
                pl_verify_logger.record(
                    sample_seq=sample_seq,
                    carla_frame=carla_frame,
                    simulation_time_s=simulation_time,
                    host_send_ns=host_send_ns,
                    host_response_ns=host_response_ns,
                    sensor=sensor,
                    input_words=input_words,
                    requested_speed_limit_kmh=requested_speed_limit,
                    weather=environment.weather,
                    rpm_level=utils.rpm_to_level(controller.current_rpm),
                    accelerator_cmd=command.throttle,
                    brake_cmd=command.brake,
                    steering_normalized=desired_steering,
                    manual_mode=fpga_manual_mode,
                    gear=0 if control.reverse else controller.current_gear + 1,
                    headlight=command.headlight,
                    hazard=command.hazard,
                    situation=situation_val,
                    fpga_result=fpga_result,
                    fault_label=control_panel.fault_label,
                )
            live_verifier.update(
                1.0 / 20.0, fpga_result, requested_speed_limit, sensor,
            )
            # MRM 진입 순간에만 1단 다운시프트를 요청한다 (R157 MRM 3번).
            # 그 뒤의 변속은 Python 의 속도 기준 로직이 이어받는다.
            # PL 의 gear 출력은 적용하지 않는 설계이므로(제어 영역 밖은 CARLA
            # 로직 그대로), MRM 다운시프트도 여기서 한 번만 걸어 준다.
            if fpga_result is not None and fpga_result.mrm and not prev_mrm:
                command.force_downshift = True
                print("[MRM] one-shot downshift requested")
            prev_mrm = bool(fpga_result is not None and fpga_result.mrm)

            if fpga_actuation_active:
                command.throttle = min(VehicleCommand.MAX_THROTTLE, fpga_result.accelerator)
                command.brake = min(VehicleCommand.MAX_BRAKE, fpga_result.brake)
                command.speed_limit = min(command.speed_limit, fpga_result.speed_limit_kmh)
                command.fpga_steering_override = fpga_result.steering_normalized

                fpga_transition_demand = fpga_result.transition_demand
                fpga_hud_warning = fpga_result.hud_warning
                fpga_mrm = fpga_result.mrm
                fpga_td_remain_sec = fpga_result.td_remain_sec
                fpga_headlight_auto_out = fpga_result.headlight
                fpga_hazard_auto_out = fpga_result.hazard

            elif fpga_result is not None:
                # Keep live warnings visible while software autonomous control
                # is driving the car and FPGA actuation is disabled.
                fpga_transition_demand = fpga_result.transition_demand
                fpga_hud_warning = fpga_result.hud_warning
                fpga_mrm = fpga_result.mrm
                fpga_td_remain_sec = fpga_result.td_remain_sec

#             obstacle_manager.update(1.0 / 30.0, ttc_result)
            if ENABLE_LEGACY_AUTO_OBSTACLES:
                obstacle_manager.update(1.0 / 20.0, ttc_result)

            # NONE 에서는 자세 위험도도 빼야 한다.  posture_result 는 command 와
            # 별개로 컨트롤러에 들어가 posture_speed_limit 을 걸기 때문에,
            # 그대로 두면 "위험도와 무관하게 제한속도만 따른다" 가 깨진다.
            controller.update(
                command,
                posture_result if control_panel.fpga_in_loop else None,
            )
            # Use the command applied in this frame for the cockpit.  The
            # earlier vehicle.get_control() snapshot belongs to the previous
            # frame and would make the hand wheel visibly lag by one tick.
            control = controller.control

            # ---------------- Engine Sound (RPM 기반 실시간 피치 조절) ----------------

            if engine_channel is not None:
                ratio = utils.clamp(controller.current_rpm / ENGINE_REFERENCE_RPM, 1.0, 8.0)
                # 소리가 나지 않던 주 원인: 너무 미세한 RPM 변화(0.02)마다 사운드를 재시작(play)해서 버퍼가 재생되기도 전에 끊겼음.
                # 이를 0.1 단위로 듬성듬성 양자화하여 끊김 현상 해결
                ratio_key = round(ratio, 1)

                if ratio_key != last_engine_ratio_key:
                    last_engine_ratio_key = ratio_key
                    if ratio_key in engine_pitch_cache_global:
                        engine_channel.play(engine_pitch_cache_global[ratio_key], loops=-1)

                engine_volume = 0.5 + 0.5 * (command.throttle / VehicleCommand.MAX_THROTTLE)
                engine_channel.set_volume(engine_volume)

            logger.log(
                simulation_time,
                sensor,
                environment,
                controller,
                perception,
                ttc_result,
                road_result,
                vision_result,
                posture_result,
                command
            )

            # control = vehicle.get_control() (Moved to top)

            
            # --- Auto/Manual Light Override ---
            if command.headlight_auto:
                command.headlight = fpga_headlight_auto_out
            else:
                command.headlight = command.manual_headlight_state

            if command.hazard_auto:
                command.hazard = fpga_hazard_auto_out
            else:
                command.hazard = command.manual_hazard_state

            # --- Apply Physical Lights to CARLA Vehicle ---
            light_state = carla.VehicleLightState.NONE
            if command.headlight:
                light_state |= carla.VehicleLightState.Position
                light_state |= carla.VehicleLightState.LowBeam
            if command.hazard:
                light_state |= carla.VehicleLightState.LeftBlinker
                light_state |= carla.VehicleLightState.RightBlinker
                
            if command.brake > 0:
                light_state |= carla.VehicleLightState.Brake
                
            if command.reverse:
                light_state |= carla.VehicleLightState.Reverse
                
            if environment.weather == Environment.FOG:
                light_state |= carla.VehicleLightState.Fog
                
            vehicle.set_light_state(carla.VehicleLightState(light_state))

            if keyboard.camera_mode == "first_person":
                draw_dashboard(
                    screen=screen,
                    sensor=sensor,
                    environment=environment,
                    controller=controller,
                    command=command,
                    vehicle_control=control,
                    fpga_result=fpga_result,
                    control_panel=control_panel,
                    actuation_active=fpga_actuation_active,
                    fpga_input_snapshot=fpga_input_snapshot,
                    input_words=input_words,
                )
            else:
                control_panel.fpga_input_toggle_rect = None

            control_panel.draw(screen)
            live_verifier.after_draw(screen)

#             pygame.display.flip()
#             clock.tick(30)
            pygame.display.flip()
            clock.tick(20)

            if live_verifier.finished:
                return "quit"

            if scenario.finished:
                return "restart"

    finally:
        print("\nCleaning up session...")

        if world_scenarios is not None:
            try:
                world_scenarios.destroy()
            except Exception:
                pass

        if camera is not None:
            try:
                camera.destroy()
            except:
                pass

        if obstacle_manager is not None:
            try:
                obstacle_manager.destroy()
            except:
                pass

        if 'sensor' in locals() and sensor is not None:
            try:
                sensor.destroy()
            except:
                pass

        if logger is not None:
            try:
                logger.close()
            except:
                pass

        if pl_verify_logger is not None:
            try:
                pl_verify_logger.close()
            except Exception:
                pass

        if fpga is not None:
            try:
                fpga.close()
            except:
                pass

        if vehicle is not None:
            try:
                vehicle.destroy()
            except:
                pass
                
        for ramp in ramp_actors:
            try:
                ramp.destroy()
            except:
                pass
                
        gc.enable()
        gc.collect()


def main():

    try:
        print("=" * 60)
        print("Connecting to CARLA...")
        print("=" * 60)

        # 오디오 초기화 및 사운드 사전 연산
        pygame.mixer.pre_init(frequency=44100, size=-16, channels=2, buffer=2048)
        pygame.init()
        pygame.font.init()
        screen = create_main_display()
        pygame.display.set_caption("CARLA FPGA Autonomous Driving")
        
        font = create_hud_font(screen)
        clock = pygame.time.Clock()

        engine_pitch_cache_global = {}
        try:
            print("[System] Pre-rendering engine sounds... (1.0 to 8.0, step 0.1)")
            if not pygame.mixer.get_init():
                pygame.mixer.init()
            engine_base = load_engine_base_array()
            for r_int in range(10, 81):
                r = round(r_int / 10.0, 1)
                engine_pitch_cache_global[r] = resample_pitch(engine_base, r)
            print("[System] 70 engine pitch levels pre-rendered successfully.")
        except Exception as e:
            print("[System] Failed to pre-render engine sounds:", e)

        client = carla.Client(HOST, PORT)
        client.set_timeout(60.0)

        # 맵 선택 인터페이스
        # This project is calibrated and verified against Town04 only.
        available_maps = ["Town04"]
        print("\n[ 사용 가능한 맵 목록 ]")
        for i, m_name in enumerate(available_maps):
            print(f"{i+1}. {m_name}")
        
        # 자동 검증 실행은 표준 입력을 기다리지 않고 환경 변수로 맵을 선택한다.
        # CARLA_MAP이 없으면 기존 대화형 맵 선택 방식을 그대로 사용한다.
        requested_map = "Town04"
        try:
            if requested_map:
                choice = requested_map
                print(f"[System] CARLA_MAP={requested_map}")
            else:
                choice = input(f"\n원하시는 맵 번호 또는 이름을 입력하세요 (엔터 시 기본값 Town04): ").strip()
            if choice == "":
                map_name = "Town04"
            elif choice in available_maps:
                map_name = choice
            elif choice + "_Opt" in available_maps:
                map_name = choice + "_Opt"
            else:
                choice_idx = int(choice) - 1
                if 0 <= choice_idx < len(available_maps):
                    map_name = available_maps[choice_idx]
                else:
                    print("잘못된 번호입니다. 기본값 Town04를 로드합니다.")
                    map_name = "Town04"
        except ValueError:
            print(f"'{choice}' 맵을 찾을 수 없습니다. 기본값 Town04를 로드합니다.")
            map_name = "Town04"
        except EOFError:
            print("입력 오류. 기본값 Town04를 로드합니다.")
            map_name = "Town04"

        current_world = client.get_world()
        current_map_name = current_world.get_map().name.rsplit("/", 1)[-1]
        if current_map_name == map_name:
            # Reuse Town04 when CARLA is already on the required map. Repeated
            # load_world() calls are slow and can leave an interrupted Windows
            # CARLA session waiting inside the map-load RPC.
            world = current_world
            print(f"\n{map_name} is already loaded; reusing the current world.")
        else:
            print(f"\nLoading {map_name}...")
            world = client.load_world(map_name)

        # A previous Python process may have terminated while owning a
        # synchronous world.  wait_for_tick() would then deadlock forever
        # because no client is issuing world.tick().  Normalize to asynchronous
        # startup before the readiness ticks, then take synchronous ownership.
        startup_settings = world.get_settings()
        if startup_settings.synchronous_mode:
            startup_settings.synchronous_mode = False
            startup_settings.fixed_delta_seconds = None
            world.apply_settings(startup_settings)

        print(f"Waiting for {map_name}...")
        for _ in range(10):
            world.wait_for_tick()
        print(f"{map_name} Ready.")

#         settings = world.get_settings()
#         settings.synchronous_mode = True
#         settings.fixed_delta_seconds = 1.0 / 30.0
#         world.apply_settings(settings)

        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 1.0 / 20.0
        world.apply_settings(settings)

        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)
        traffic_manager.set_global_distance_to_leading_vehicle(3.0)
        traffic_manager.global_percentage_speed_difference(10.0)

        bp_lib = world.get_blueprint_library()

        map_manager = MapManager(world)
        keyboard = KeyboardController()
        if env_flag("CARLA_FIRST_PERSON", default=False):
            keyboard.camera_mode = "first_person"
        control_panel = ControlPanel()

        status = "restart"

        status = "restart"

        while status == "restart":
            status = run_session(
                world, map_manager, screen, font, clock, keyboard,
                traffic_manager, engine_pitch_cache_global, control_panel,
            )

    except KeyboardInterrupt:
        print("\nUser Exit.")

    except Exception as e:
        import traceback
        traceback.print_exc()
        print("\nERROR")
        print(e)

    finally:
        try:
            if 'world' in locals() and world is not None:
                settings = world.get_settings()
                settings.synchronous_mode = False
                settings.fixed_delta_seconds = None
                world.apply_settings(settings)
        except:
            pass
            
        pygame.quit()
        print("Finished.")


if __name__ == "__main__":
    main()
