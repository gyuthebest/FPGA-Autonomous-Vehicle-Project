"""Apply control-panel risk scenarios to the live CARLA world.

The control panel only stores user intent.  This class owns every CARLA actor
and weather/physics side effect created by the risk controls, so a session can
remove them deterministically during CLEAR, restart, or program shutdown.
"""

from __future__ import annotations

import math
import random

import carla

import utils
from environment import Environment


class WorldScenarioController:
    MAX_OBSTACLE_DISTANCE_M = 190.0
    DRIVER_REACTION_TIME_S = 0.7
    CONSERVATIVE_BRAKING_MPS2 = 7.5
    STOPPING_MARGIN_M = 10.0
    # A visible safety command must last long enough to be observed over the
    # 20 Hz PL sample stream.  Posture direction is held even longer so gyro
    # and lateral risk are sustained conditions, not one-frame impulses.
    MIN_RISK_HOLD_S = 2.0
    POSTURE_DIRECTION_HOLD_S = 3.0

    def __init__(self, world, ego_vehicle, weather_manager, control_panel):
        self.world = world
        self.ego = ego_vehicle
        self.weather_manager = weather_manager
        self.panel = control_panel
        self.map = world.get_map()

        self._collision_actor = None
        self._friction_actor = None
        self._rough_actors = []
        self._rough_points = []
        self._rough_hits = set()

        self._seen_collision_request = control_panel.collision_request
        self._seen_reset_request = control_panel.scenario_reset_request
        self._surface_key = None
        self._roughness_key = None
        self._weather_key = None
        self._posture_elapsed = 0.0
        self._posture_direction = 1.0
        self._posture_was_active = False
        self._road_impact_hold_remaining = 0.0
        self._vehicle_mass = float(self.ego.get_physics_control().mass)
        self._scenario_sensor_frame = 0

    def update(self, dt):
        self._road_impact_hold_remaining = max(
            0.0, self._road_impact_hold_remaining - float(dt)
        )
        if self.panel.scenario_reset_request != self._seen_reset_request:
            self._seen_reset_request = self.panel.scenario_reset_request
            self._destroy_actor("_collision_actor")
            self._destroy_actor("_friction_actor")
            self._destroy_roughness()
            self._surface_key = None
            self._roughness_key = None
            self._weather_key = None
            self.panel.collision_active = False

        if self.panel.collision_request != self._seen_collision_request:
            self._seen_collision_request = self.panel.collision_request
            self._spawn_safe_obstacle()

        self._sync_surface()
        self._sync_weather()
        self._sync_roughness()
        self._apply_roughness_impulse()
        self._apply_posture(dt)

    @property
    def driven_channels(self):
        """지금 시나리오가 직접 값을 써 넣고 있는 센서 채널 이름.

        `apply_sensor_conditions`는 이미 triangle로 1 LSB 움직임을 넣는다.
        여기에 SensorNoiseModel의 기본 잡음까지 더해지면 두 잡음원이 합쳐져
        온도처럼 NOISE_THRESHOLD_1이 작은 채널(2 LSB)에서 여유가 사라진다.
        main.py가 이 집합을 노이즈 모델의 skip으로 넘겨 둘 중 하나만
        적용되게 한다.
        """
        driven = set()
        if self.panel.road_surface != "dry":
            driven.update({"temperature", "humidity"})
        if int(self.panel.visibility_risk) > 0:
            driven.add("lux")
        if self._road_impact_hold_remaining > 0.0:
            driven.add("accel_z")
        posture = self.panel.posture
        if posture["roll"]:
            driven.add("gyro_x")
        if posture["yaw"]:
            driven.add("gyro_z")
        if posture["lateral"]:
            driven.add("accel_y")
        return driven

    def apply_sensor_conditions(self, sensor):
        """Make selected physical scenarios deterministic at the PL boundary.

        CARLA physics remains active and visible.  These values model the
        sustained sensor response that the physical condition should produce,
        so a risk is present for enough consecutive 20 Hz samples to exercise
        the PL control path reliably on every run.
        """
        self._scenario_sensor_frame += 1
        # A slow triangular one-LSB motion models normal quantization movement
        # without crossing PL jump/noise thresholds.  Truly constant synthetic
        # values otherwise trip the PL stuck check after only 15 samples and
        # incorrectly promote WET to ICE and DIM to DARK.
        triangle = (-2, -1, 0, 1, 2, 1, 0, -1)[self._scenario_sensor_frame % 8]

        collision_tier = int(self.panel.collision_tier)
        if collision_tier:
            # Use a 10 m/s closing speed and select a point strictly inside
            # each TTC band in risk_types.sv.  The physical actor remains at
            # its independently calculated safe stopping distance.
            sensor.approach_speed = 10.0
            sensor.distance = {
                1: 35.0,   # 3.0 s < TTC <= 4.0 s
                2: 25.0,   # 2.0 s < TTC <= 3.0 s
                3: 18.0,   # 1.5 s < TTC <= 2.0 s
                4: 12.0,   # TTC <= 1.5 s
            }[collision_tier]

        # 온도 LSB는 0.1 degC 이므로 triangle 1스텝이 1 LSB가 되도록 0.1을 곱한다.
        # 습도 LSB는 1 %라 그대로 쓴다.  PL이 보는 LSB 움직임은 이전과 같다.
        temp_step = triangle * 0.1

        surface = self.panel.road_surface
        if surface == "wet":
            # 12.0 degC / 85 % -> temp > 0 이고 humidity >= 70 이므로 WET
            sensor.temperature, sensor.humidity = 12.0 + temp_step, 85.0 + triangle
        elif surface == "ice":
            # -5.0 degC(raw -50) / 85 % -> humidity < 90 이라 ICE
            sensor.temperature, sensor.humidity = -5.0 + temp_step, 85.0 + triangle
        elif surface == "black_ice":
            # -8.0 degC(raw -80 <= -50) / 95 %(>= 90) -> BLACK ICE.
            # 이전 값 -60.0 degC는 온도 scale이 1.0이던 시절 raw -60을 만들기
            # 위한 보정값이었고, 0.1 degC 스케일에서는 raw -600이 되어 range
            # 하한(-500)을 벗어난다.
            sensor.temperature, sensor.humidity = -8.0 + temp_step, 95.0 + triangle

        visibility = int(self.panel.visibility_risk)
        if visibility > 0:
            sensor.lux = ((10000.0 if visibility <= 33 else (500.0 if visibility <= 66 else 20.0))
                          + triangle)

        if self._road_impact_hold_remaining > 0.0:
            roughness = int(self.panel.roughness)
            if roughness <= 33:
                sensor.accel_z = 15.2 + triangle * 0.1  # >= 0.5 g: ROUGH
            elif roughness <= 66:
                sensor.accel_z = 20.0 + triangle * 0.1  # >= 1.0 g: SEVERE
            else:
                sensor.accel_z = -12.0 + triangle * 0.1 # reaches 2 g: EXTREME

        posture = self.panel.posture
        if posture["roll"]:
            sensor.gyro_x = self._posture_direction * (
                0.72 + 0.25 * posture["roll"] / 100.0 + triangle * 0.002
            )
        if posture["yaw"]:
            sensor.gyro_z = self._posture_direction * (
                (0.56 if posture["yaw"] <= 50 else 1.10) + triangle * 0.002
            )
        if posture["lateral"]:
            sensor.accel_y = self._posture_direction * (
                (5.1 if posture["lateral"] <= 50 else 8.1) + triangle * 0.02
            )

    def destroy(self):
        self._destroy_actor("_collision_actor")
        self._destroy_actor("_friction_actor")
        self._destroy_roughness()

    def _spawn_safe_obstacle(self):
        self._destroy_actor("_collision_actor")
        self.panel.collision_active = False

        velocity = self.ego.get_velocity()
        speed_mps = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
        stopping_distance = (
            speed_mps * self.DRIVER_REACTION_TIME_S
            + speed_mps ** 2 / (2.0 * self.CONSERVATIVE_BRAKING_MPS2)
            + self.STOPPING_MARGIN_M
        )
        minimum = max(25.0, stopping_distance)
        if minimum > self.MAX_OBSTACLE_DISTANCE_M:
            self.panel.collision_status = (
                f"Not spawned: required safe distance {minimum:.1f} m exceeds 190 m"
            )
            print(f"[SCENARIO] {self.panel.collision_status}")
            return

        maximum = min(self.MAX_OBSTACLE_DISTANCE_M, max(minimum, minimum + 50.0))
        distance = random.uniform(minimum, maximum)
        ego_waypoint = self.map.get_waypoint(
            self.ego.get_location(), project_to_road=True,
            lane_type=carla.LaneType.Driving,
        )
        candidates = ego_waypoint.next(distance) if ego_waypoint else []
        if not candidates:
            self.panel.collision_status = "Not spawned: no road waypoint ahead"
            print(f"[SCENARIO] {self.panel.collision_status}")
            return

        ego_yaw = self.ego.get_transform().rotation.yaw
        target = utils.select_aligned_waypoint(candidates, ego_yaw)
        if target is None:
            self.panel.collision_status = (
                "Not spawned: no lane-aligned waypoint ahead (junction/curve)"
            )
            print(f"[SCENARIO] {self.panel.collision_status}")
            return
        transform = carla.Transform(target.transform.location, target.transform.rotation)
        transform.location.z += 0.35

        blueprints = list(self.world.get_blueprint_library().filter("vehicle.*"))
        random.shuffle(blueprints)
        for blueprint in blueprints[:8]:
            actor = self.world.try_spawn_actor(blueprint, transform)
            if actor is None:
                continue
            actor.set_autopilot(False)
            actor.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
            self._collision_actor = actor
            self.panel.collision_active = True
            # A just-spawned CARLA actor reports Location(0,0,0) until the
            # next server tick. Use the requested spawn transform here; it is
            # the authoritative placement for this synchronous frame.
            actual = self.ego.get_location().distance(transform.location)
            if actual > 200.0:
                actor.destroy()
                self._collision_actor = None
                self.panel.collision_status = (
                    "Not spawned: ego moved before the safe placement completed"
                )
                print(f"[SCENARIO] {self.panel.collision_status}")
                return
            self.panel.collision_status = (
                f"Obstacle at {actual:.1f} m (safe minimum {minimum:.1f} m)"
            )
            print(f"[SCENARIO] {self.panel.collision_status}")
            return

        self.panel.collision_status = "Not spawned: target position is occupied"
        print(f"[SCENARIO] {self.panel.collision_status}")

    def _sync_surface(self):
        surface = self.panel.road_surface
        if surface == self._surface_key:
            return
        self._surface_key = surface
        self._destroy_actor("_friction_actor")

        friction = {
            "dry": None,
            "wet": 1.20,
            "ice": 0.55,
            "black_ice": 0.20,
        }[surface]
        if friction is None:
            print("[SCENARIO] Road surface: DRY")
            return

        try:
            blueprint = self.world.get_blueprint_library().find("static.trigger.friction")
            blueprint.set_attribute("friction", str(friction))
            blueprint.set_attribute("extent_x", "500")
            blueprint.set_attribute("extent_y", "500")
            blueprint.set_attribute("extent_z", "20")
            transform = carla.Transform(self.ego.get_location(), carla.Rotation())
            transform.location.z -= 2.0
            self._friction_actor = self.world.spawn_actor(blueprint, transform)
            print(f"[SCENARIO] Road surface: {surface.upper()} (friction={friction:.2f})")
        except Exception as exc:
            print(f"[SCENARIO] Friction volume failed: {exc}")

    def _sync_weather(self):
        key = (
            self.panel.weather,
            self.panel.road_surface,
            int(self.panel.visibility_risk),
        )
        if key == self._weather_key:
            return
        self._weather_key = key

        weather_name, surface, visibility = key
        presets = {
            "clear": dict(cloudiness=0, precipitation=0, deposits=0,
                          wetness=0, fog=0, wind=5,
                          code=Environment.CLEAR, temperature=22.0, humidity=30.0),
            "rain": dict(cloudiness=95, precipitation=80, deposits=70,
                         wetness=90, fog=8, wind=20,
                         code=Environment.RAIN, temperature=15.0, humidity=85.0),
            "fog": dict(cloudiness=40, precipitation=0, deposits=0,
                        wetness=10, fog=90, wind=5,
                        code=Environment.FOG, temperature=12.0, humidity=90.0),
            # CARLA renders snow with its precipitation/deposit parameters;
            # the Environment.SNOW code supplies the FPGA semantic label.
            "snow": dict(cloudiness=90, precipitation=60, deposits=80,
                         wetness=30, fog=20, wind=30,
                         code=Environment.SNOW, temperature=-3.0, humidity=80.0),
        }
        preset = presets[weather_name].copy()

        if surface == "wet":
            preset["wetness"] = max(preset["wetness"], 85)
            preset["deposits"] = max(preset["deposits"], 40)
            preset["temperature"], preset["humidity"] = 12.0, 85.0
        elif surface == "ice":
            preset["wetness"] = max(preset["wetness"], 55)
            preset["deposits"] = max(preset["deposits"], 55)
            preset["temperature"], preset["humidity"] = -5.0, 85.0
        elif surface == "black_ice":
            preset["wetness"] = max(preset["wetness"], 80)
            # -8.0 degC = raw -80 <= -50 (BLACK ICE 임계). 온도 LSB 0.1 degC 기준.
            preset["temperature"], preset["humidity"] = -8.0, 95.0

        fog_density = max(float(preset["fog"]), float(visibility))
        normalized = visibility / 100.0
        fog_distance = max(2.0, 1000.0 * (1.0 - normalized) ** 3)
        if weather_name == "fog":
            fog_distance = min(fog_distance, 5.0)

        current = self.world.get_weather()
        weather = carla.WeatherParameters(
            cloudiness=float(preset["cloudiness"]),
            precipitation=float(preset["precipitation"]),
            precipitation_deposits=float(preset["deposits"]),
            wetness=float(preset["wetness"]),
            fog_density=fog_density,
            fog_distance=fog_distance,
            fog_falloff=max(0.1, 1.0 - normalized * 0.8),
            wind_intensity=float(preset["wind"]),
            sun_altitude_angle=current.sun_altitude_angle,
            sun_azimuth_angle=current.sun_azimuth_angle,
        )
        self.world.set_weather(weather)
        self.weather_manager.current = weather_name
        self.weather_manager.weather_code = (
            Environment.FOG
            if weather_name == "clear" and visibility > 0
            else preset["code"]
        )
        self.weather_manager.temperature = preset["temperature"]
        self.weather_manager.humidity = preset["humidity"]
        print(
            f"[SCENARIO] Weather={weather_name.upper()}, visibility risk={visibility}%, "
            f"surface={surface.upper()}"
        )

    def _sync_roughness(self):
        # Rebuild at 10-percent steps so dragging the slider does not issue a
        # large burst of actor spawn/destroy RPCs.
        roughness = int(round(self.panel.roughness / 10.0) * 10)
        if roughness == self._roughness_key:
            return
        self._roughness_key = roughness
        self._destroy_roughness()
        if roughness <= 0:
            print("[SCENARIO] Road roughness: OFF")
            return

        start_distance = 20.0
        strip_count = max(1, min(5, int(math.ceil(roughness / 20.0))))
        ego_waypoint = self.map.get_waypoint(
            self.ego.get_location(), project_to_road=True,
            lane_type=carla.LaneType.Driving,
        )
        if ego_waypoint is None:
            return

        try:
            blueprint = self.world.get_blueprint_library().find("static.prop.box01")
        except Exception as exc:
            print(f"[SCENARIO] Road roughness blueprint unavailable: {exc}")
            return

        for index in range(strip_count):
            candidates = ego_waypoint.next(start_distance + index * 10.0)
            if not candidates:
                continue
            waypoint = candidates[0]
            transform = waypoint.transform
            right = transform.get_right_vector()
            self._rough_points.append(carla.Location(
                transform.location.x, transform.location.y, transform.location.z
            ))
            for offset in (-1.5, -0.5, 0.5, 1.5):
                location = carla.Location(
                    x=transform.location.x + right.x * offset,
                    y=transform.location.y + right.y * offset,
                    z=transform.location.z - 0.48,
                )
                actor = self.world.try_spawn_actor(
                    blueprint, carla.Transform(location, transform.rotation)
                )
                if actor is not None:
                    self._rough_actors.append(actor)
        print(f"[SCENARIO] Road roughness: {roughness}% ({len(self._rough_points)} strips)")

    def _apply_roughness_impulse(self):
        if not self._rough_points or not self._roughness_key:
            return
        ego_location = self.ego.get_location()
        for index, location in enumerate(self._rough_points):
            if index in self._rough_hits or ego_location.distance(location) > 3.0:
                continue
            mass = self.ego.get_physics_control().mass
            vertical_speed = 0.35 + 2.65 * self._roughness_key / 100.0
            self.ego.add_impulse(carla.Vector3D(z=mass * vertical_speed))
            self._rough_hits.add(index)
            self._road_impact_hold_remaining = self.MIN_RISK_HOLD_S
            print(f"[SCENARIO] Road impact impulse: {self._roughness_key}%")

    def _apply_posture(self, dt):
        values = self.panel.posture
        if not any(values.values()):
            if self._posture_was_active:
                self.ego.set_target_angular_velocity(carla.Vector3D())
            self._posture_elapsed = 0.0
            self._posture_direction = 1.0
            self._posture_was_active = False
            return
        self._posture_was_active = True
        self._posture_elapsed += dt
        if self._posture_elapsed >= self.POSTURE_DIRECTION_HOLD_S:
            self._posture_elapsed -= self.POSTURE_DIRECTION_HOLD_S
            self._posture_direction *= -1.0

        transform = self.ego.get_transform()
        forward = transform.get_forward_vector()
        right = transform.get_right_vector()
        roll_rate = (0.0 if not values["roll"] else
                     self._posture_direction * (42.0 + 18.0 * values["roll"] / 100.0))
        yaw_rate = (0.0 if not values["yaw"] else
                    self._posture_direction * (35.0 + 35.0 * values["yaw"] / 100.0))
        angular = carla.Vector3D(
            x=forward.x * roll_rate,
            y=forward.y * roll_rate,
            z=yaw_rate,
        )
        self.ego.set_target_angular_velocity(angular)

        if values["lateral"]:
            acceleration = 5.2 + 3.6 * values["lateral"] / 100.0
            force = self._vehicle_mass * acceleration * self._posture_direction
            self.ego.add_force(
                carla.Vector3D(x=right.x * force, y=right.y * force, z=0.0)
            )

    def _destroy_actor(self, attribute):
        actor = getattr(self, attribute)
        if actor is not None:
            try:
                actor.destroy()
            except Exception:
                pass
        setattr(self, attribute, None)

    def _destroy_roughness(self):
        for actor in self._rough_actors:
            try:
                actor.destroy()
            except Exception:
                pass
        self._rough_actors.clear()
        self._rough_points.clear()
        self._rough_hits.clear()
