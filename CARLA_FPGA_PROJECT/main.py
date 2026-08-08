import random
import carla
import pygame
import numpy as np

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

import os

HOST = "127.0.0.1"
PORT = 2000
VEHICLE_ID = "vehicle.ford.mustang"

WEATHER_CYCLE_INTERVAL = 30.0

def hud_color(text):
    if any(k in text for k in ("HIGH", "DANGER", "EXTREME", "BLACK_ICE", "VERY_DARK")):
        return (255, 60, 60)
    if any(k in text for k in ("MEDIUM", "CAUTION", "SEVERE", "ICE", "DARK", "FOG", "SNOW", "ROUGH", "RAIN")):
        return (255, 220, 0)
    if any(k in text for k in ("LOW", "SAFE", "DRY", "NORMAL", "CLEAR", "BRIGHT", "DIM")):
        return (100, 255, 100)
    if text.startswith("["):
        return (120, 220, 255)
    return (0, 255, 0)


def draw_hud_column(screen, font, lines, x, y, line_height):
    for item in lines:
        if isinstance(item, tuple):
            text, color = item
        else:
            text, color = item, hud_color(item)
        surface = font.render(text, True, color)
        screen.blit(surface, (x, y))
        y += line_height
    return y

ENGINE_SOUND_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets", "sounds", "engine_loop.wav")

# 음원 파일을 녹음/제작할 때 기준이 된 RPM
# (이 RPM에서 재생하면 피치 변화 없이 원본 그대로 들린다)
ENGINE_REFERENCE_RPM = 800


def load_engine_base_array():
    """
    엔진 루프 음원을 로드해 float 배열로 반환한다.
    (리샘플링 연산은 float 상태에서 하고, Sound 생성 시점에 int16으로 변환)
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

def run_session(world, map_manager, screen, font, clock, keyboard, traffic_manager):
    """
    차량 스폰부터 세션 종료까지 한 번의 주행을 실행한다.
    반환값: 'restart' | 'quit'
    """

    vehicle = None
    camera = None
    obstacle_manager = None
    logger = None

    try:
        bp_lib = world.get_blueprint_library()
        try:
            vehicle_bp = bp_lib.find(VEHICLE_ID)
        except:
            vehicle_bp = bp_lib.filter("vehicle.*")[0]

        spawn = random.choice(world.get_map().get_spawn_points())

        vehicle = world.spawn_actor(vehicle_bp, spawn)
        vehicle.set_autopilot(False)

        camera = CameraManager(world, vehicle)
        sensor = SensorManager(vehicle)

        environment_manager = EnvironmentManager(map_manager)
        obstacle_manager = ObstacleManager(world, vehicle, map_manager, traffic_manager)
        perception_manager = PerceptionManager(world, vehicle)
        controller = VehicleController(world, vehicle)
        scenario = ScenarioManager()
        weather_manager = WeatherManager(world)
        logger = CSVLogger()

        controller = VehicleController(world, vehicle)
        scenario = ScenarioManager()
        weather_manager = WeatherManager(world)
        logger = CSVLogger()

        
        try:
            if not pygame.mixer.get_init():
                pygame.mixer.init()

            mixer_info = pygame.mixer.get_init()
            print(f"[Audio] Mixer initialized: freq={mixer_info[0]}, format={mixer_info[1]}, channels={mixer_info[2]}")
            print(f"[Audio] Loading sound from: {ENGINE_SOUND_PATH}")

            if not os.path.isfile(ENGINE_SOUND_PATH):
                raise FileNotFoundError(f"엔진 사운드 파일을 찾을 수 없습니다: {ENGINE_SOUND_PATH}")

            engine_channel = pygame.mixer.Channel(0)
            engine_base_array = load_engine_base_array()
            engine_pitch_cache = {}
            last_engine_ratio_key = None
            engine_channel.play(resample_pitch(engine_base_array, 1.0), loops=-1)
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

        print("Vehicle Spawned.")
        print()
        print("========== CONTROL ==========")
        print("M   : AUTO / MANUAL")
        print("W   : Throttle / Reverse-Brake")
        print("S   : Brake / Reverse-Throttle")
        print("A/D : Steering")
        print("R   : Restart")
        print("ESC : Exit")
        print("=============================")
        print()

        while True:

            status = keyboard.poll_system_events()

            if status in ("quit", "restart"):
                return status

            world.tick()

            sensor.update()
            sensor.rpm = utils.rpm_to_level(controller.current_rpm)
            sensor.weather = weather_manager.weather_code
            sensor.temperature = weather_manager.temperature
            sensor.humidity = weather_manager.humidity

            environment = environment_manager.update(sensor, controller.upcoming_turn_speed_limit)
            perception = perception_manager.update()

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

            if weather_timer >= WEATHER_CYCLE_INTERVAL:
                weather_timer = 0.0
                weather_index = (weather_index + 1) % len(weather_cycle)
                weather_cycle[weather_index]()

            simulation_time += 1.0 / 20.0

            screen.fill((0, 0, 0))

            if camera.image is not None:
                frame = camera.image
                frame = frame[:, :, ::-1]
                surface = pygame.surfarray.make_surface(frame.swapaxes(0, 1))
                screen.blit(surface, (0, 0))

            manual_command = VehicleCommand()
            keyboard.update(manual_command, sensor.speed)

            ttc_result = ttc_logic.update(sensor, perception)
            road_result = road_logic.update(environment, sensor)
            vision_result = vision_logic.update(environment, sensor)
            posture_result = posture_logic.update(sensor)

            if keyboard.manual_mode:
                command = manual_command
            else:
                command = fusion_logic.fuse(
                    ttc=ttc_result,
                    posture=posture_result,
                    road=road_result,
                    vision=vision_result
                )

            command.manual_mode = keyboard.manual_mode
            command.autonomous_control = (not keyboard.manual_mode)

#             obstacle_manager.update(1.0 / 30.0, ttc_result)
            obstacle_manager.update(1.0 / 20.0, ttc_result)

            controller.update(command, posture_result)

            # ---------------- Engine Sound (RPM 기반 실시간 피치 조절) ----------------

            if engine_channel is not None:
                ratio = utils.clamp(controller.current_rpm / ENGINE_REFERENCE_RPM, 1.0, 8.0)
                # 소리가 나지 않던 주 원인: 너무 미세한 RPM 변화(0.02)마다 사운드를 재시작(play)해서 버퍼가 재생되기도 전에 끊겼음.
                # 이를 0.1 단위로 듬성듬성 양자화하여 끊김 현상 해결
                ratio_key = round(ratio, 1)

                if ratio_key != last_engine_ratio_key:
                    last_engine_ratio_key = ratio_key

                    if ratio_key not in engine_pitch_cache:
                        engine_pitch_cache[ratio_key] = resample_pitch(engine_base_array, ratio_key)

                        if len(engine_pitch_cache) > 30:
                            engine_pitch_cache.pop(next(iter(engine_pitch_cache)))

                    engine_channel.play(engine_pitch_cache[ratio_key], loops=-1)

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

            control = vehicle.get_control()

            # ========================================================
            # [FPGA AXI REGISTER PACKING - TEST]
            # ========================================================
            sample_seq += 1
            
            # [추가된 부분: 파이썬 기반 루프 소요 시간 측정 (Hardware ILA 대체)]
            current_time = time.perf_counter()
            gap_ms = (current_time - last_time) * 1000.0
            last_time = current_time

            # 첫 번째 프레임(초기화 딜레이)은 무시하고, 역대 최장 시간이 갱신되면 출력
            if sample_seq > 1 and gap_ms > gap_max_ms:
                gap_max_ms = gap_ms
                print(f"[*] 최대 지터 갱신! 새로운 gap_max: {gap_max_ms:.2f} ms (프레임 seq: {sample_seq})")

            def to_signed(val, bits):
                val = int(val)
                if val < 0:
                    val = (1 << bits) + val
                return val & ((1 << bits) - 1)

            def to_unsigned(val, bits):
                return int(val) & ((1 << bits) - 1)

            # --- 센서 스케일링 (x100, x1000) ---
            accel_x = to_signed(sensor.accel_x * 100, 12)
            accel_y = to_signed(sensor.accel_y * 100, 12)
            accel_z = to_signed(sensor.accel_z * 100, 12)
            
            speed_x = to_signed(sensor.speed_x * 100, 14)
            speed_y = to_signed(sensor.speed_y * 100, 14)
            speed_z = to_signed(sensor.speed_z * 100, 14)

            gyro_x = to_signed(sensor.gyro_x * 1000, 16)
            gyro_y = to_signed(sensor.gyro_y * 1000, 16)
            gyro_z = to_signed(sensor.gyro_z * 1000, 16)

            incline_x = to_signed(sensor.incline_x * 100, 16)
            incline_y = to_signed(sensor.incline_y * 100, 16)
            incline_z = to_signed(sensor.incline_z * 100, 16)

            distance = to_unsigned(sensor.distance * 100, 15)
            app_speed = to_signed(sensor.approach_speed * 100, 13)
            
            speed_limit = to_unsigned(environment.speed_limit * 100, 13)
            lux = to_unsigned(sensor.lux, 18)
            temperature = to_signed(sensor.temperature, 11)
            humidity = to_unsigned(sensor.humidity, 7)

            weather_val = to_unsigned(environment.weather, 2)
            rpm_val = to_unsigned(utils.rpm_to_level(controller.current_rpm), 2)
            gear_val = to_unsigned(0 if control.reverse else controller.current_gear+1, 2)
            
            steering_val = to_signed(control.steer * 100, 8)
            accelerator_val = to_unsigned(command.throttle, 4)
            brake_val = to_unsigned(command.brake, 4)
            
            manual_mode = 1 if command.manual_mode else 0
            headlight = 1 if command.headlight else 0
            hazard = 1 if command.hazard else 0

            # --- AXI Register Packing ---
            slv_reg0 = (humidity << 24) | (accel_y << 12) | accel_x
            slv_reg1 = (gear_val << 30) | (rpm_val << 28) | (weather_val << 26) | (speed_x << 12) | accel_z
            slv_reg2 = (hazard << 30) | (headlight << 29) | (manual_mode << 28) | (speed_z << 14) | speed_y
            slv_reg3 = (gyro_y << 16) | gyro_x
            slv_reg4 = (app_speed << 16) | gyro_z
            slv_reg5 = (incline_y << 16) | incline_x
            slv_reg6 = (distance << 16) | incline_z
            slv_reg7 = (speed_limit << 18) | lux
            slv_reg8 = (steering_val << 19) | (brake_val << 15) | (accelerator_val << 11) | temperature
            
            slv_reg9 = to_unsigned(sample_seq, 32)

            # TODO: Add physical FPGA Write here
            # ========================================================

            mode = "MANUAL" if command.manual_mode else "AUTO"

            risk_map = {0: "LOW", 1: "MEDIUM", 2: "HIGH"}

            weather_map = {
                Environment.CLEAR: "CLEAR",
                Environment.FOG: "FOG",
                Environment.RAIN: "RAIN",
                Environment.SNOW: "SNOW",
            }

            # ---------------- Front Dist: 가까우면 빨강, 멀면 초록 ----------------

            if ttc_result is not None and ttc_result.distance_over_range:
                front_dist_text = "OVER 200m"
                front_dist_color = (100, 255, 100)  # 멀다 = 안전 = 초록
            else:
                d = ttc_result.distance if ttc_result else perception.front_distance
                front_dist_text = f"{d:5.1f} m"

                if d < 20:
                    front_dist_color = (255, 60, 60)   # 가깝다 = 위험 = 빨강
                elif d < 50:
                    front_dist_color = (255, 220, 0)
                else:
                    front_dist_color = (100, 255, 100)

            # ---------------- Speed Limit % (도로제한속도 대비 제어로직 제한 비율) ----------------

            if environment.speed_limit > 0:
                speed_limit_pct = min(100.0, (command.speed_limit / environment.speed_limit) * 100.0)
            else:
                speed_limit_pct = 100.0


            confidence_color_map = {
                "HIGH": (100, 255, 100),
                "MEDIUM": (255, 220, 0),
                "LOW": (255, 60, 60),
            }
            confidence_color = confidence_color_map.get(scenario.confidence, (0, 255, 0))

            col_left = [
                "===== DECISION MONITOR =====",
                "",
                "[SYSTEM]",
                f"Mode          : {mode}",

                "",
                "[INPUT]",
                f"Speed         : {sensor.speed:5.1f} km/h",
                f"Road Limit    : {environment.speed_limit:5.1f} km/h",
                (f"Front Dist    : {front_dist_text}", front_dist_color),
                f"Lux           : {sensor.lux:6.1f}",
                f"Temperature   : {sensor.temperature:5.1f} C",
                f"Humidity      : {sensor.humidity:5.1f} %",
                f"Weather       : {weather_map.get(environment.weather, environment.weather)}",

                "",
                f"Zone          : {map_manager.get_zone(vehicle.get_location())}",

                "",
                "[FUSION]",
                f"Final Risk    : {risk_map.get(command.final_risk, command.final_risk)}",
                (f"Confidence    : {scenario.confidence}", confidence_color),
            ]

            thr_bar = "#" * command.throttle + "-" * (10 - command.throttle)
            brk_bar = "#" * command.brake + "-" * (10 - command.brake)

            col_right = [
                "[LOGIC]",
                f"TTC Risk      : {'-' if ttc_result is None else risk_map.get(ttc_result.final_risk, ttc_result.final_risk)}",
                f"Road Surface  : {'-' if road_result is None else road_result.surface_grade}",
                f"Road Shock    : {'-' if road_result is None else road_result.shock_grade}",
                f"Vision Lux    : {'-' if vision_result is None else vision_result.lux_grade}",
                f"Vision Weather: {'-' if vision_result is None else vision_result.weather_grade}",
                f"Posture Roll  : {'-' if posture_result is None else posture_result.roll_grade}",
                f"Posture Yaw   : {'-' if posture_result is None else posture_result.yaw_grade}",
                f"Posture Accel : {'-' if posture_result is None else posture_result.lateral_grade}",

                "",
                "[OUTPUT]",
                f"Throttle [{thr_bar}] {command.throttle:2d}/10",
                f"Speed Limit   : {speed_limit_pct:5.1f} %",
                f"Brake    [{brk_bar}] {command.brake:2d}/10",
                f"Steer Limit   : {command.steering_rate_limit:3d}%",
                f"Steering      : {control.steer:5.2f}",
                f"Gear          : {'R' if control.reverse else 'D' + str(controller.current_gear+1)}",
                f"RPM           : {controller.current_rpm:4d}",
                f"Headlight     : {'ON' if command.headlight else 'OFF'}",
                f"Hazard        : {'ON' if command.hazard else 'OFF'}",
                f"Manual Mode   : {'YES' if command.manual_mode else 'NO'}",
            ]

            LINE_H = 16
            HUD_X1 = 650
            HUD_X2 = 965

            hud_h = max(len(col_left), len(col_right)) * LINE_H + 20

            hud_background = pygame.Surface((625, hud_h))
            hud_background.set_alpha(150)
            hud_background.fill((0, 0, 0))
            screen.blit(hud_background, (HUD_X1 - 5, 5))

            draw_hud_column(screen, font, col_left, HUD_X1, 10, LINE_H)
            draw_hud_column(screen, font, col_right, HUD_X2, 10, LINE_H)

#             pygame.display.flip()
#             clock.tick(30)
            pygame.display.flip()
            clock.tick(20)

            if scenario.finished:
                return "restart"

    finally:
        print("\nCleaning up session...")

        try:
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)
        except:
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

        if vehicle is not None:
            try:
                vehicle.destroy()
            except:
                pass


def main():

    try:
        print("=" * 60)
        print("Connecting to CARLA...")
        print("=" * 60)

        # 오디오 초기화 (윈도우 기본 드라이버 자동 선택)
        pygame.mixer.pre_init(frequency=44100, size=-16, channels=2, buffer=2048)
        pygame.init()

        screen = pygame.display.set_mode((1280, 400))
        pygame.display.set_caption("CARLA FPGA Simulator")

        font = pygame.font.SysFont("consolas", 13)
        clock = pygame.time.Clock()

        client = carla.Client(HOST, PORT)
        client.set_timeout(60.0)

        # 맵 선택 인터페이스
        available_maps = [m.split('/')[-1] for m in client.get_available_maps()]
        print("\n[ 사용 가능한 맵 목록 ]")
        for i, m_name in enumerate(available_maps):
            print(f"{i+1}. {m_name}")
        
        try:
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

        print(f"\nLoading {map_name}...")
        world = client.load_world(map_name)

        print(f"Waiting for {map_name}...")
        for _ in range(100):
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

        status = "restart"

        status = "restart"

        while status == "restart":
            status = run_session(world, map_manager, screen, font, clock, keyboard, traffic_manager)

    except KeyboardInterrupt:
        print("\nUser Exit.")

    except Exception as e:
        import traceback
        traceback.print_exc()
        print("\nERROR")
        print(e)

    finally:
        pygame.quit()
        print("Finished.")


if __name__ == "__main__":
    main()