"""Deterministic checks that CARLA test controls reach the intended PL tiers."""

from types import SimpleNamespace
import unittest

import carla

import utils
from main import (
    SITUATION_NORMAL, SITUATION_OBSTACLE, SITUATION_POSTURE,
    SITUATION_STOPPED, SITUATION_WEATHER,
    classify_situation, should_apply_fpga_output,
)
from sensor_noise import SensorNoiseModel
from world_scenario_controller import WorldScenarioController


class _EgoStub:
    def __init__(self):
        self.angular_commands = []
        self.forces = []

    def get_transform(self):
        return carla.Transform(carla.Location(), carla.Rotation())

    def set_target_angular_velocity(self, value):
        self.angular_commands.append(value)

    def add_force(self, value):
        self.forces.append(value)


def _sensor():
    return SimpleNamespace(
        temperature=22.0, humidity=30.0, lux=30000.0,
        accel_y=0.0, accel_z=9.8, gyro_x=0.0, gyro_z=0.0,
    )


def _controller(surface="dry", visibility=0, roughness=0,
                roll=0, yaw=0, lateral=0):
    controller = WorldScenarioController.__new__(WorldScenarioController)
    controller.panel = SimpleNamespace(
        collision_tier=0,
        road_surface=surface,
        visibility_risk=visibility,
        roughness=roughness,
        posture={"roll": roll, "yaw": yaw, "lateral": lateral},
    )
    controller._posture_direction = 1.0
    controller._posture_elapsed = 0.0
    controller._posture_was_active = False
    controller._road_impact_hold_remaining = 0.0
    controller._vehicle_mass = 1500.0
    controller._scenario_sensor_frame = 0
    controller.ego = _EgoStub()
    return controller


class ScenarioPLAlignmentTest(unittest.TestCase):
    def test_all_collision_tiers(self):
        for selected_tier in range(1, 5):
            with self.subTest(tier=selected_tier):
                sensor = SimpleNamespace(distance=200.0, approach_speed=0.0)
                controller = _controller()
                controller.panel.collision_tier = selected_tier
                controller.apply_sensor_conditions(sensor)
                distance_raw = round(sensor.distance * 100.0)
                closing_raw = round(sensor.approach_speed * 100.0)
                tier = (4 if distance_raw <= closing_raw + (closing_raw >> 1)
                        else 3 if distance_raw <= (closing_raw << 1)
                        else 2 if distance_raw <= closing_raw * 3
                        else 1 if distance_raw <= (closing_raw << 2) else 0)
                self.assertEqual(tier, selected_tier)

    def test_all_road_surface_tiers(self):
        expected = {
            "dry": 0,
            "wet": 1,
            "ice": 2,
            "black_ice": 3,
        }
        for surface, tier in expected.items():
            with self.subTest(surface=surface):
                sensor = _sensor()
                _controller(surface=surface).apply_sensor_conditions(sensor)
                # risk_types.sv의 임계값(-50, 0)은 PL raw 단위다.  온도 LSB는
                # 0.1 degC이므로 물리값을 그대로 비교하면 안 되고 먼저
                # 양자화해야 한다(거리 tier 시험과 같은 방식).
                temperature_raw = round(sensor.temperature * 10.0)
                humidity_raw = round(sensor.humidity)
                classified = (3 if temperature_raw <= -50 and humidity_raw >= 90
                              else 2 if temperature_raw <= 0 and humidity_raw >= 70
                              else 1 if humidity_raw >= 70 else 0)
                self.assertEqual(classified, tier)

    def test_all_light_visibility_tiers(self):
        for slider, expected_tier in ((0, 0), (20, 1), (50, 2), (90, 3)):
            with self.subTest(slider=slider):
                sensor = _sensor()
                _controller(visibility=slider).apply_sensor_conditions(sensor)
                tier = (0 if sensor.lux >= 20000 else 1 if sensor.lux >= 1000
                        else 2 if sensor.lux >= 50 else 3)
                self.assertEqual(tier, expected_tier)

    def test_all_road_impact_tiers_are_held(self):
        for roughness, expected_tier in ((20, 1), (50, 2), (90, 3)):
            with self.subTest(roughness=roughness):
                sensor = _sensor()
                controller = _controller(roughness=roughness)
                controller._road_impact_hold_remaining = controller.MIN_RISK_HOLD_S
                controller.apply_sensor_conditions(sensor)
                raw_accel_z = round(sensor.accel_z * 100.0)
                net = abs(raw_accel_z - 980)
                tier = 3 if net >= 1960 else 2 if net >= 980 else 1 if net >= 490 else 0
                self.assertEqual(tier, expected_tier)
                self.assertGreaterEqual(controller.MIN_RISK_HOLD_S, 2.0)

    def test_posture_sensor_tiers(self):
        cases = (
            ({"roll": 10}, "gyro_x", 1),
            ({"yaw": 25}, "gyro_z", 1),
            ({"yaw": 75}, "gyro_z", 2),
            ({"lateral": 25}, "accel_y", 1),
            ({"lateral": 75}, "accel_y", 2),
        )
        for settings, field, expected in cases:
            with self.subTest(settings=settings):
                controller = _controller(**settings)
                sensor = _sensor()
                controller.apply_sensor_conditions(sensor)
                raw = abs(round(getattr(sensor, field) * (1000 if field.startswith("gyro") else 100)))
                if field == "gyro_x":
                    tier = int(raw >= 698)
                elif field == "gyro_z":
                    tier = 2 if raw >= 1047 else 1 if raw >= 524 else 0
                else:
                    tier = 2 if raw >= 784 else 1 if raw >= 490 else 0
                self.assertEqual(tier, expected)

    def test_posture_is_continuous_and_direction_holds_three_seconds(self):
        controller = _controller(roll=100, yaw=100, lateral=100)
        for _ in range(59):
            controller._apply_posture(0.05)
        self.assertEqual(controller._posture_direction, 1.0)
        self.assertEqual(len(controller.ego.angular_commands), 59)
        self.assertEqual(len(controller.ego.forces), 59)
        # Binary floating-point may represent 60 * 0.05 just below 3.0; the
        # next 20 Hz frame must reverse, never an earlier frame.
        controller._apply_posture(0.05)
        self.assertEqual(controller._posture_direction, 1.0)
        controller._apply_posture(0.05)
        self.assertEqual(controller._posture_direction, -1.0)
        self.assertGreaterEqual(controller.POSTURE_DIRECTION_HOLD_S, 3.0)

    def test_td_and_mrm_keep_fpga_authority_after_button_release(self):
        panel = SimpleNamespace(
            apply_fpga_output=True, intervention_scenario_active=False,
        )
        result = SimpleNamespace(transition_demand=False, mrm=False)
        self.assertFalse(should_apply_fpga_output(panel, result))
        panel.intervention_scenario_active = True
        self.assertTrue(should_apply_fpga_output(panel, result))
        panel.intervention_scenario_active = False
        result.transition_demand = True
        self.assertTrue(should_apply_fpga_output(panel, result))
        result.transition_demand = False
        result.mrm = True
        self.assertTrue(should_apply_fpga_output(panel, result))
        panel.apply_fpga_output = False
        self.assertFalse(should_apply_fpga_output(panel, result))


def _waypoint(yaw, is_junction=False):
    return SimpleNamespace(
        is_junction=is_junction,
        transform=SimpleNamespace(rotation=SimpleNamespace(yaw=yaw)),
    )


class SpawnAlignmentTest(unittest.TestCase):
    """장애물이 진행 방향과 정렬된 차선에만 생성되는지 검증."""

    def test_picks_smallest_heading_error(self):
        candidates = [_waypoint(12.0), _waypoint(3.0), _waypoint(-20.0)]
        chosen = utils.select_aligned_waypoint(candidates, 0.0)
        self.assertEqual(chosen.transform.rotation.yaw, 3.0)

    def test_rejects_perpendicular_branch(self):
        """교차로 분기만 남으면 스폰하지 않는다.

        기존 min() 방식은 90도 어긋난 waypoint도 무조건 반환해서, 앞차가
        도로를 가로질러 서 있는 장면을 만들었다.
        """
        candidates = [_waypoint(90.0), _waypoint(-88.0)]
        self.assertIsNone(utils.select_aligned_waypoint(candidates, 0.0))

    def test_rejects_junction_waypoints(self):
        candidates = [_waypoint(2.0, is_junction=True)]
        self.assertIsNone(utils.select_aligned_waypoint(candidates, 0.0))

    def test_wraparound_heading_is_not_a_false_rejection(self):
        """359도와 1도는 2도 차이다. 모듈러 처리 확인."""
        chosen = utils.select_aligned_waypoint([_waypoint(359.0)], 1.0)
        self.assertIsNotNone(chosen)
        self.assertAlmostEqual(utils.heading_error_deg(359.0, 1.0), 2.0)

    def test_opposite_direction_is_rejected(self):
        """역주행 차선(180도)은 후보가 될 수 없다."""
        self.assertIsNone(utils.select_aligned_waypoint([_waypoint(180.0)], 0.0))

    def test_empty_candidate_list(self):
        self.assertIsNone(utils.select_aligned_waypoint([], 0.0))
        self.assertIsNone(utils.select_aligned_waypoint(None, 0.0))


class SensorNoiseIntegrationTest(unittest.TestCase):
    """기본 센서 잡음이 안전 제약을 지키는지 검증."""

    def test_distance_and_approach_speed_are_untouched(self):
        """무표적 sentinel(20000/0)이 깨지면 distance 진단이 되살아난다."""
        model = SensorNoiseModel(enabled=True)
        sensor = SimpleNamespace(
            distance=200.0, approach_speed=0.0,
            accel_x=0.0, accel_y=0.0, accel_z=9.81,
            gyro_x=0.0, gyro_y=0.0, gyro_z=0.0,
            temperature=22.0, humidity=40.0, lux=30000.0,
        )
        for _ in range(50):
            model.apply(sensor)
            self.assertEqual(sensor.distance, 200.0)
            self.assertEqual(sensor.approach_speed, 0.0)

    def test_noise_never_leaves_rtl_range(self):
        model = SensorNoiseModel(enabled=True)
        for _ in range(200):
            sensor = SimpleNamespace(
                accel_x=0.0, accel_y=0.0, accel_z=9.81,
                gyro_x=0.0, gyro_y=0.0, gyro_z=0.0,
                temperature=22.0, humidity=100.0, lux=130000.0,
                distance=200.0, approach_speed=0.0,
            )
            model.apply(sensor)
            # humidity range 상한은 100, lux는 130000
            self.assertLessEqual(sensor.humidity, 100.0)
            self.assertGreaterEqual(sensor.humidity, 0.0)
            self.assertLessEqual(sensor.lux, 130000.0)
            self.assertLessEqual(abs(sensor.accel_z), 16.0)

    def test_disabled_model_only_keeps_environment_floor(self):
        """잡음을 꺼도 온도/습도/조도에는 바닥 진동이 남는다.

        이전 시험은 "비활성이면 아무것도 바뀌지 않는다"를 검사했다.  그 정책
        아래에서는 환경 채널이 완전히 일정해져 stuck 이 확정된다.  실측:
        CARLA_SENSOR_NOISE=0 개루프 3분에서 temperature 100 % /
        humidity 100 % / lux 93.3 % 가 DEGRADED 였다.

        환경 값이 일정한 것은 물리적으로 정상이므로 stuck 판정 쪽이 사각지대이고,
        그 설계 판단이 서기 전까지는 계측 잡음 바닥을 항상 깔아 둔다.
        """
        model = SensorNoiseModel(enabled=False)
        sensor = SimpleNamespace(temperature=22.0, humidity=40.0, lux=30000.0,
                                 accel_x=0.0, accel_y=0.0, accel_z=9.81,
                                 gyro_x=0.0, gyro_y=0.0, gyro_z=0.0)
        model.apply(sensor)

        # 운동 채널은 그대로여야 한다 (바닥 진동은 환경 채널 전용).
        for attr in ("accel_x", "accel_y", "gyro_x", "gyro_y", "gyro_z"):
            self.assertEqual(getattr(sensor, attr), 0.0, attr)
        self.assertEqual(sensor.accel_z, 9.81)

    def test_environment_floor_prevents_stuck_without_tripping_noise(self):
        """바닥 진동이 stuck 을 막으면서 jump/noise 임계는 넘지 않아야 한다.

        each_sensor_check 기준 (온도/습도):
          stuck  : delta 가 0 이면 누적 -> delta 는 매 표본 0 이 아니어야 한다
          noise  : 10표본 |delta| 합 <= NOISE_THRESHOLD_1(2) * 10 = 20
                   10창 부호 반전 <= NOISE_THRESHOLD_2 = 7
          jump   : |delta - prev_delta| <= JUMP_THRESHOLD(5)
        """
        scales = {"temperature": 10.0, "humidity": 1.0, "lux": 1.0}
        model = SensorNoiseModel(enabled=False)
        raw = {name: [] for name in scales}
        for _ in range(30):
            sensor = SimpleNamespace(temperature=22.0, humidity=40.0,
                                     lux=30000.0, accel_x=0.0, accel_y=0.0,
                                     accel_z=9.81, gyro_x=0.0, gyro_y=0.0,
                                     gyro_z=0.0)
            model.apply(sensor)
            for name, scale in scales.items():
                raw[name].append(round(getattr(sensor, name) * scale))

        for name, values in raw.items():
            deltas = [values[i] - values[i - 1] for i in range(1, len(values))]
            self.assertNotIn(0, deltas, f"{name}: delta 가 0 이면 stuck 이 쌓인다")
            window = deltas[:10]
            flips = sum(1 for i in range(1, len(window))
                        if (window[i] < 0) != (window[i - 1] < 0))
            self.assertLessEqual(flips, 7, f"{name}: 부호 반전 과다")
            self.assertLessEqual(max(abs(d - deltas[i])
                                     for i, d in enumerate(deltas[1:])), 5,
                                 f"{name}: 2차 차분이 jump 임계를 넘는다")
            if name in ("temperature", "humidity"):
                self.assertLessEqual(sum(abs(d) for d in window), 20,
                                     f"{name}: noise 임계 초과")

    def test_model_is_deterministic(self):
        """캡처 재생 비교를 위해 같은 표본 수는 항상 같은 파형이어야 한다."""
        def run():
            model = SensorNoiseModel(enabled=True)
            sensor = SimpleNamespace(
                accel_x=0.0, accel_y=0.0, accel_z=9.81,
                gyro_x=0.0, gyro_y=0.0, gyro_z=0.0,
                temperature=22.0, humidity=40.0, lux=30000.0,
            )
            values = []
            for _ in range(30):
                model.apply(sensor)
                values.append((sensor.temperature, sensor.gyro_z))
            return values

        self.assertEqual(run(), run())


class SituationEncodingTest(unittest.TestCase):
    """situation 3비트 인코딩이 사양과 일치하는지.

    000 정지 / 001 장애물 등장 / 010 자세변화 / 011 날씨 변화 / 100 정상.
    PL의 jump/consistency 마스크가 이 값에 의존하므로 정확해야 한다.
    """

    @staticmethod
    def _sensor(speed_x=20.0, distance=200.0, gx=0.0, gy=0.0, gz=0.0):
        return SimpleNamespace(speed_x=speed_x, distance=distance,
                               gyro_x=gx, gyro_y=gy, gyro_z=gz)

    def test_stopped(self):
        sensor = self._sensor(speed_x=0.1)
        self.assertEqual(classify_situation(sensor, 200.0, False),
                         SITUATION_STOPPED)

    def test_normal_driving(self):
        sensor = self._sensor(speed_x=20.0)
        self.assertEqual(classify_situation(sensor, 200.0, False),
                         SITUATION_NORMAL)

    def test_obstacle_appears_only_on_sentinel_crossing(self):
        sensor = self._sensor(distance=60.0)
        self.assertEqual(classify_situation(sensor, 200.0, False),
                         SITUATION_OBSTACLE)
        # 이미 추적 중이면 사건이 아니다.
        self.assertEqual(classify_situation(sensor, 61.0, False),
                         SITUATION_NORMAL)

    def test_posture_covers_all_three_axes(self):
        """사양은 각속도 x/y/z 세 축 모두다."""
        for axis in ("gx", "gy", "gz"):
            with self.subTest(axis=axis):
                sensor = self._sensor(**{axis: 0.5})
                self.assertEqual(classify_situation(sensor, 200.0, False),
                                 SITUATION_POSTURE)

    def test_posture_threshold_boundary(self):
        self.assertEqual(
            classify_situation(self._sensor(gz=0.34), 200.0, False),
            SITUATION_NORMAL)
        self.assertEqual(
            classify_situation(self._sensor(gz=0.341), 200.0, False),
            SITUATION_POSTURE)

    def test_weather_has_highest_priority(self):
        sensor = self._sensor(speed_x=0.0, distance=60.0, gz=0.9)
        self.assertEqual(classify_situation(sensor, 200.0, True),
                         SITUATION_WEATHER)

    def test_event_beats_state(self):
        """정지 중에 자세변화가 나면 자세변화가 우선한다."""
        sensor = self._sensor(speed_x=0.0, gz=0.9)
        self.assertEqual(classify_situation(sensor, 200.0, False),
                         SITUATION_POSTURE)

    def test_all_codes_fit_three_bits(self):
        for code in (SITUATION_STOPPED, SITUATION_OBSTACLE, SITUATION_POSTURE,
                     SITUATION_WEATHER, SITUATION_NORMAL):
            self.assertTrue(0 <= code <= 7)


if __name__ == "__main__":
    unittest.main()
