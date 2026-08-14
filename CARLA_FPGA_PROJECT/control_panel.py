"""Interactive fault/risk injection panel for the CARLA FPGA demo."""

from __future__ import annotations

import random
from typing import Dict, Iterable, List, Optional, Set, Tuple

import pygame


SENSORS: Tuple[Tuple[str, str], ...] = (
    ("distance", "Distance"),
    ("approach_speed", "Closing Speed"),
    ("accel_x", "Accel X"),
    ("accel_y", "Accel Y"),
    ("accel_z", "Accel Z"),
    ("gyro_x", "Gyro X"),
    ("gyro_y", "Gyro Y"),
    ("gyro_z", "Gyro Z"),
    ("temperature", "Temperature"),
    ("humidity", "Humidity"),
    ("lux", "Illuminance"),
)

ALL_CHECKS = ("range", "jump", "stuck", "noise", "consistency", "timeout")
CONSISTENCY_SENSORS = {
    "distance", "approach_speed", "accel_x", "accel_y", "accel_z",
    "gyro_x", "gyro_y", "gyro_z",
}
# distance는 15비트 부호 없음(0..327.67 m)이고 PL의 range 상한은 200.00 m라서
# 200 m 초과 값은 표현 가능하며 정상적으로 range fault를 만든다.  이전에는
# build_input_words가 200.0 m에서 포화시켜 이 경로를 막고 있었을 뿐이다.
RANGE_SENSORS = set(dict(SENSORS))

RISK_SECTIONS: Tuple[Tuple[str, str], ...] = (
    ("collision", "Collision"),
    ("road_surface", "Road Surface"),
    ("road_impact", "Road Impact"),
    ("visibility", "Visibility"),
    ("weather", "Weather"),
    ("posture", "Posture"),
)


def decode_risk_word(word: int) -> Dict[str, int]:
    """Decode AXI read register 9 according to risk_types.sv."""
    value = int(word) & 0xFFFF
    return {
        "collision": value & 0x7,
        "road_surface": (value >> 3) & 0x3,
        "road_impact": (value >> 5) & 0x3,
        "visibility_light": (value >> 7) & 0x3,
        "visibility_weather": (value >> 9) & 0x3,
        "roll": (value >> 11) & 0x1,
        "yaw": (value >> 12) & 0x3,
        "lateral": (value >> 14) & 0x3,
    }


RISK_TEXT = {
    "collision": ("SAFE", "CAUTION", "DANGER", "CRITICAL", "EMERGENCY"),
    "road_surface": ("DRY", "WET", "ICE", "BLACK ICE"),
    "road_impact": ("NORMAL", "ROUGH", "SEVERE", "EXTREME"),
    "visibility_light": ("BRIGHT", "DIM", "DARK", "VERY DARK"),
    "visibility_weather": ("CLEAR", "FOG", "RAIN", "SNOW"),
    "roll": ("SAFE", "DANGER"),
    "yaw": ("SAFE", "CAUTION", "DANGER"),
    "lateral": ("SAFE", "CAUTION", "DANGER"),
}


def risk_text(category: str, level: int) -> str:
    names = RISK_TEXT[category]
    return names[level] if 0 <= int(level) < len(names) else f"LEVEL {level}"


class FaultInjector:
    """Stateful sensor and risk injection applied before AXI packing."""

    def __init__(self) -> None:
        self.sensor_faults: Set[Tuple[str, str]] = set()
        self.risk_faults: Set[str] = set()
        self._stuck_values: Dict[str, float] = {}
        self._baseline: Dict[str, float] = {}
        self._frame = 0

    def toggle_sensor_fault(self, sensor_name: str, check: str) -> bool:
        key = (sensor_name, check)
        if key in self.sensor_faults:
            self.sensor_faults.remove(key)
            self._stuck_values.pop(sensor_name, None)
            return False
        if check == "timeout":
            self.sensor_faults = {
                item for item in self.sensor_faults if item[1] != "timeout"
            }
        self.sensor_faults.add(key)
        return True

    def toggle_risk(self, risk_name: str) -> bool:
        if risk_name in self.risk_faults:
            self.risk_faults.remove(risk_name)
            return False
        self.risk_faults.add(risk_name)
        return True

    @property
    def drop_sample(self) -> bool:
        return any(check == "timeout" for _sensor, check in self.sensor_faults)

    @property
    def frozen_channels(self) -> Set[str]:
        """고장이 주입되어 기본 측정 잡음을 덧씌우면 안 되는 채널.

        SensorNoiseModel.apply(..., skip=...)에 그대로 넘긴다.  주입된
        stuck 상수값에 잡음이 더해지면 고장 자체가 사라지기 때문이다.
        """
        return {name for name, check in self.sensor_faults if check != "timeout"}

    @property
    def fault_label(self) -> str:
        labels = [f"{sensor}:{check}" for sensor, check in sorted(self.sensor_faults)]
        labels.extend(f"risk:{name}" for name in sorted(self.risk_faults))
        return "+".join(labels) if labels else "none"

    def supported_checks(self, sensor_name: str) -> List[str]:
        result = ["jump", "stuck", "noise", "timeout"]
        if sensor_name in RANGE_SENSORS:
            result.insert(0, "range")
        if sensor_name in CONSISTENCY_SENSORS:
            result.insert(-1, "consistency")
        return result

    def apply(self, sensor) -> None:
        self._frame += 1
        # CARLA's no-target protocol value (200 m, 0 m/s closing) masks every
        # distance diagnostic in PL by design.  A distance-fault button must
        # therefore establish a valid tracked-target baseline first.
        if any(
            name == "distance" or (name == "approach_speed" and check == "stuck")
            for name, check in self.sensor_faults
        ):
            sensor.distance = min(float(sensor.distance), 80.0)
            sensor.approach_speed = 0.08
        self._baseline = {
            name: float(getattr(sensor, name)) for name, _label in SENSORS
        }
        for sensor_name, check in sorted(self.sensor_faults):
            if check == "timeout":
                continue
            original = float(getattr(sensor, sensor_name))
            if check == "range":
                value = self._range_value(sensor_name)
            elif check == "jump":
                if sensor_name.startswith("accel_"):
                    # Alternate between two legal in-range measurements.  The
                    # resulting residual exceeds the 20 m/s^2 jump threshold
                    # without accidentally turning this into a range fault.
                    value = max(-15.5, min(15.5, self._alternating(15.0)))
                else:
                    value = original + self._alternating(self._jump_amplitude(sensor_name))
            elif check == "stuck":
                stuck_seed = 0.30 if sensor_name.startswith("gyro_") else original
                value = self._stuck_values.setdefault(sensor_name, stuck_seed)
                self._drive_stuck_trigger(sensor, sensor_name)
            elif check == "noise":
                amplitude = self._noise_amplitude(sensor_name)
                # A bounded alternating noisy signal guarantees that the
                # ten-sample PL mean-deviation/sign-flip window is exercised;
                # a purely random draw can occasionally miss its threshold.
                value = original + self._alternating(amplitude)
                if sensor_name.startswith("accel_"):
                    value = max(-15.5, min(15.5, value))
            elif check == "consistency":
                if sensor_name in {"accel_y", "accel_z"}:
                    # Remove the grade mask so the dynamics relationship is
                    # testable regardless of where Town04 spawned the car.
                    sensor.incline_x = 0.0
                    sensor.incline_y = 0.0
                value = original + self._alternating(self._consistency_amplitude(sensor_name))
            else:
                continue
            setattr(sensor, sensor_name, value)

        self._apply_risk_faults(sensor)

    def _alternating(self, amplitude: float) -> float:
        jitter = random.uniform(0.85, 1.15)
        return amplitude * jitter * (-1.0 if self._frame & 1 else 1.0)

    @staticmethod
    def _range_value(name: str) -> float:
        values = {
            # 250 m -> 25000 LSB. PL RANGE_THRESHOLD_MAX(20000)를 넘고
            # 15비트 필드 상한(32767)에는 걸리지 않는다.
            "distance": 250.0,
            "approach_speed": 40.8,
            "accel_x": 18.0, "accel_y": -18.0, "accel_z": 18.0,
            "gyro_x": 18.0, "gyro_y": -18.0, "gyro_z": 18.0,
            # 70.0 degC -> raw 700 > RANGE_MAX(600). 온도 LSB 0.1 degC 기준.
            "temperature": 70.0, "humidity": 110.0, "lux": 150000.0,
        }
        if name in {"humidity", "lux", "distance"}:
            return values[name]
        return values[name] * random.choice((-1.0, 1.0))

    @staticmethod
    def _jump_amplitude(name: str) -> float:
        return {
            "distance": 4.0, "approach_speed": 2.0,
            "accel_x": 25.0, "accel_y": 25.0, "accel_z": 25.0,
            "gyro_x": 2.0, "gyro_y": 2.0, "gyro_z": 2.0,
            # 1.2 degC -> raw 12 > JUMP_THRESHOLD(5)
            "temperature": 1.2, "humidity": 12.0, "lux": 30000.0,
        }[name]

    @staticmethod
    def _noise_amplitude(name: str) -> float:
        return {
            "distance": 1.5, "approach_speed": 0.8,
            "accel_x": 10.0, "accel_y": 10.0, "accel_z": 10.0,
            "gyro_x": 0.5, "gyro_y": 0.5, "gyro_z": 0.5,
            # 0.6 degC -> raw 6, 10표본 평균이 NOISE_THRESHOLD_1(2)를 넘김
            "temperature": 0.6, "humidity": 6.0, "lux": 12000.0,
        }[name]

    @staticmethod
    def _consistency_amplitude(name: str) -> float:
        return {
            "distance": 20.0, "approach_speed": 2.0,
            "accel_x": 3.0, "accel_y": 8.0, "accel_z": 8.0,
            "gyro_x": 1.0, "gyro_y": 1.0, "gyro_z": 1.0,
        }[name]

    def _apply_risk_faults(self, sensor) -> None:
        if "collision" in self.risk_faults:
            sensor.distance = random.uniform(3.0, 6.0)
            sensor.approach_speed = random.uniform(8.0, 12.0)
        if "road_surface" in self.risk_faults:
            # -8.0 degC -> raw -80 <= -50 (BLACK ICE) 이고 range 하한(-500) 안쪽.
            sensor.temperature = -8.0
            sensor.humidity = 95.0
        if "road_impact" in self.risk_faults:
            sensor.speed_x = max(10.0, abs(float(sensor.speed_x)))
            sensor.accel_z = random.choice((-12.0, 30.0))
        if "visibility_light" in self.risk_faults:
            sensor.lux = random.uniform(10.0, 30.0)
        if "visibility_weather" in self.risk_faults:
            sensor.weather = 3
        if "roll" in self.risk_faults:
            sensor.gyro_x = random.choice((-0.9, 0.9))
        if "yaw" in self.risk_faults:
            sensor.gyro_z = random.choice((-1.2, 1.2))
        if "lateral" in self.risk_faults:
            sensor.accel_y = random.choice((-9.0, 9.0))

    def _drive_stuck_trigger(self, sensor, name: str) -> None:
        """Excite the independent reference used by triggered stuck checks."""
        sign = -1.0 if self._frame & 1 else 1.0
        if name == "distance":
            # The corrected PL trigger uses the non-zero closing-speed value,
            # not its delta. Keep the independent reference valid and stable.
            sensor.approach_speed = 0.08
        elif name == "approach_speed":
            # Keep the reference moving without tripping the distance jump
            # diagnostic that would mask this cross-channel stuck test.
            sensor.distance = max(1.0, self._baseline["distance"] + sign * 0.08)
        elif name.startswith("accel_"):
            axis = name[-1]
            setattr(sensor, f"speed_{axis}", sign * 5.0)
        elif name.startswith("gyro_"):
            axis = name[-1]
            setattr(sensor, f"incline_{axis}", sign * 8.0)


class ControlPanel:
    def __init__(self) -> None:
        self.open = False
        self.tab = "sensor"
        self.selected_sensor = "distance"
        # ARMED by default.  main.py still keeps normal CARLA autonomous
        # control unless a deliberate sensor-fault/risk scenario is active.
        self.apply_fpga_output = True
        self.injector = FaultInjector()
        self.risk_section = "collision"
        self.collision_request = 0
        self.scenario_reset_request = 0
        self.collision_status = "Ready"
        self.collision_active = False
        self.collision_tier = 0
        self.road_surface = "dry"
        self.roughness = 0
        self.visibility_risk = 0
        self.weather = "clear"
        self.posture = {"roll": 0, "yaw": 0, "lateral": 0}
        self.last_reliability_word: Optional[int] = None
        self.show_fpga_inputs = False
        self.fpga_input_toggle_rect: Optional[pygame.Rect] = None
        self._buttons: List[Tuple[pygame.Rect, str, str]] = []
        self._sliders: List[Tuple[pygame.Rect, str]] = []
        self._dragging_slider: Optional[str] = None
        self._font: Optional[pygame.font.Font] = None
        self._small_font: Optional[pygame.font.Font] = None

    def set_fpga_result(self, result) -> None:
        if result is not None:
            self.last_reliability_word = int(result.reliability_word)

    def selected_reliability_state(self) -> str:
        if self.last_reliability_word is None:
            return "NO FPGA DATA"
        index = next(i for i, item in enumerate(SENSORS) if item[0] == self.selected_sensor)
        state = (self.last_reliability_word >> (2 * index)) & 0x3
        return ("NORMAL", "DEGRADED", "INVALID", "RESERVED")[state]

    def handle_event(self, event: pygame.event.Event) -> bool:
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if (self.fpga_input_toggle_rect is not None and
                    self.fpga_input_toggle_rect.collidepoint(event.pos)):
                self.show_fpga_inputs = not self.show_fpga_inputs
                print(
                    f"[HUD] FPGA input monitor: "
                    f"{'ON' if self.show_fpga_inputs else 'OFF'}"
                )
                return True
            for rect, action, value in reversed(self._buttons):
                if not rect.collidepoint(event.pos):
                    continue
                if action == "panel":
                    self.open = not self.open
                elif action == "tab":
                    self.tab = value
                elif action == "sensor":
                    self.selected_sensor = value
                elif action == "check":
                    enabled = self.injector.toggle_sensor_fault(self.selected_sensor, value)
                    print(f"[INJECT] {self.selected_sensor}/{value}: {'ON' if enabled else 'OFF'}")
                elif action == "risk_section":
                    self.risk_section = value
                elif action == "collision_spawn":
                    self.collision_request += 1
                    self.collision_status = "Spawn requested"
                elif action == "collision_tier":
                    self.collision_tier = int(value)
                elif action == "surface":
                    self.road_surface = value
                elif action == "weather":
                    self.weather = value
                elif action == "fpga_apply":
                    self.apply_fpga_output = not self.apply_fpga_output
                    print(f"[CONTROL] Apply FPGA output: {'ON' if self.apply_fpga_output else 'OFF'}")
                elif action == "clear":
                    self.clear_scenarios()
                    print("[INJECT] all injections and scenarios cleared")
                return True
            for rect, name in self._sliders:
                if rect.inflate(0, 14).collidepoint(event.pos):
                    self._dragging_slider = name
                    self._set_slider(name, event.pos[0], rect)
                    return True

        if event.type == pygame.MOUSEMOTION and self._dragging_slider:
            for rect, name in self._sliders:
                if name == self._dragging_slider:
                    self._set_slider(name, event.pos[0], rect)
                    return True

        if event.type == pygame.MOUSEBUTTONUP and event.button == 1 and self._dragging_slider:
            self._dragging_slider = None
            return True
        return False

    @property
    def fault_label(self) -> str:
        labels = [] if self.injector.fault_label == "none" else [self.injector.fault_label]
        if self.collision_active:
            labels.append("collision")
        if self.collision_tier:
            labels.append(f"collision_tier:{self.collision_tier}")
        if self.road_surface != "dry":
            labels.append(f"surface:{self.road_surface}")
        if self.roughness:
            labels.append(f"roughness:{self.roughness}")
        if self.visibility_risk:
            labels.append(f"visibility:{self.visibility_risk}")
        if self.weather != "clear":
            labels.append(f"weather:{self.weather}")
        labels.extend(f"{name}:{value}" for name, value in self.posture.items() if value)
        return "+".join(labels) if labels else "none"

    @property
    def intervention_scenario_active(self) -> bool:
        """Whether the demonstration currently requests FPGA intervention."""
        return bool(
            self.injector.sensor_faults
            or self.collision_active
            or self.collision_tier
            or self.road_surface != "dry"
            or self.roughness
            or self.visibility_risk
            or self.weather != "clear"
            or any(self.posture.values())
        )

    def clear_scenarios(self) -> None:
        self.injector.sensor_faults.clear()
        self.injector.risk_faults.clear()
        self.injector._stuck_values.clear()
        self.road_surface = "dry"
        self.roughness = 0
        self.visibility_risk = 0
        self.weather = "clear"
        self.posture = {"roll": 0, "yaw": 0, "lateral": 0}
        self.collision_status = "Cleared"
        self.collision_active = False
        self.collision_tier = 0
        self.scenario_reset_request += 1

    def _set_slider(self, name: str, mouse_x: int, rect: pygame.Rect) -> None:
        value = round(max(0.0, min(1.0, (mouse_x - rect.x) / rect.width)) * 100)
        if name == "roughness":
            self.roughness = value
        elif name == "visibility":
            self.visibility_risk = value
        else:
            self.posture[name] = value

    def draw(self, screen: pygame.Surface) -> None:
        self._font = pygame.font.SysFont("consolas", max(14, screen.get_height() // 55), bold=True)
        self._small_font = pygame.font.SysFont("consolas", max(12, screen.get_height() // 66))
        self._buttons = []
        self._sliders = []
        margin = 12
        toggle_w, toggle_h = 150, 34
        toggle = pygame.Rect(screen.get_width() - toggle_w - margin,
                             screen.get_height() - toggle_h - margin,
                             toggle_w, toggle_h)
        self._button(screen, toggle, "CONTROL PANEL", self.open, "panel", "")
        if not self.open:
            return

        panel_w = min(650, screen.get_width() - 24)
        panel_h = min(405, screen.get_height() - 70)
        panel = pygame.Rect(screen.get_width() - panel_w - margin,
                            screen.get_height() - panel_h - toggle_h - 20,
                            panel_w, panel_h)
        bg = pygame.Surface(panel.size, pygame.SRCALPHA)
        bg.fill((10, 14, 20, 235))
        screen.blit(bg, panel.topleft)
        pygame.draw.rect(screen, (80, 190, 255), panel, 2, border_radius=8)

        x, y = panel.x + 12, panel.y + 10
        tab_w = 145
        self._button(screen, pygame.Rect(x, y, tab_w, 30), "SENSOR FAULTS",
                     self.tab == "sensor", "tab", "sensor")
        self._button(screen, pygame.Rect(x + tab_w + 8, y, tab_w, 30), "RISK CONTROL",
                     self.tab == "risk", "tab", "risk")
        self._button(screen, pygame.Rect(panel.right - 238, y, 140, 30),
                     f"FPGA: {'ARMED' if self.apply_fpga_output else 'BYPASS'}",
                     self.apply_fpga_output, "fpga_apply", "")
        self._button(screen, pygame.Rect(panel.right - 90, y, 78, 30),
                     "CLEAR", False, "clear", "")
        y += 42

        if self.tab == "sensor":
            self._draw_sensor_tab(screen, panel, x, y)
        else:
            self._draw_risk_tab(screen, panel, x, y)

    def _draw_sensor_tab(self, screen, panel, x, y) -> None:
        button_w = (panel.width - 48) // 4
        for index, (name, label) in enumerate(SENSORS):
            row, col = divmod(index, 4)
            rect = pygame.Rect(x + col * (button_w + 8), y + row * 32, button_w, 27)
            active = name == self.selected_sensor
            self._button(screen, rect, label, active, "sensor", name, small=True)

        y += 3 * 32 + 8
        title = self._font.render(
            f"{dict(SENSORS)[self.selected_sensor]} checks  |  FPGA: {self.selected_reliability_state()}",
            True, (220, 235, 245),
        )
        screen.blit(title, (x, y))
        y += 28
        checks = self.injector.supported_checks(self.selected_sensor)
        check_w = (panel.width - 48) // 3
        for index, check in enumerate(checks):
            row, col = divmod(index, 3)
            rect = pygame.Rect(x + col * (check_w + 8), y + row * 35, check_w, 30)
            active = (self.selected_sensor, check) in self.injector.sensor_faults
            label = "TIMEOUT (GLOBAL)" if check == "timeout" else check.upper()
            self._button(screen, rect, label, active, "check", check, small=True)

        if self.selected_sensor == "distance":
            note = "Range unavailable: AXI max equals 200 m range ceiling."
            surface = self._small_font.render(note, True, (255, 200, 90))
            screen.blit(surface, (x, panel.bottom - 24))

    def _draw_risk_tab(self, screen, panel, x, y) -> None:
        button_w = (panel.width - 64) // 3
        for index, (name, label) in enumerate(RISK_SECTIONS):
            row, col = divmod(index, 3)
            rect = pygame.Rect(x + col * (button_w + 8), y + row * 35, button_w, 29)
            self._button(screen, rect, label, name == self.risk_section,
                         "risk_section", name, small=True)
        y += 78

        if self.risk_section == "collision":
            self._button(screen, pygame.Rect(x, y, 270, 38), "SPAWN SAFE OBSTACLE",
                         False, "collision_spawn", "")
            self._choice_row(
                screen, x, y + 48, "collision_tier",
                ((0, "TRACKED"), (1, "CAUTION"), (2, "DANGER"),
                 (3, "CRITICAL"), (4, "EMERGENCY")),
                self.collision_tier, panel.width - 24,
            )
            self._text(screen, x, y + 90, f"Status: {self.collision_status}")
            self._text(screen, x, y + 114,
                       "Tier buttons hold a PL test measurement while the visible obstacle stays safely placed.")
        elif self.risk_section == "road_surface":
            self._choice_row(screen, x, y, "surface",
                             (("dry", "DRY"), ("wet", "WET"),
                              ("ice", "ICE"), ("black_ice", "BLACK ICE")),
                             self.road_surface, panel.width - 24)
            self._text(screen, x, y + 48,
                       "Changes road wetness, temperature/humidity and the CARLA friction volume.")
        elif self.risk_section == "road_impact":
            self._slider(screen, x, y + 28, panel.width - 36, "roughness",
                         "Road roughness", self.roughness)
            self._text(screen, x, y + 78,
                       "Creates bumps ahead; each impact is held for 2 s at the PL sensor boundary.")
        elif self.risk_section == "visibility":
            self._slider(screen, x, y + 28, panel.width - 36, "visibility",
                         "Visibility risk", self.visibility_risk)
            self._text(screen, x, y + 78,
                       "0 = clear view, 100 = dense short-range fog.")
        elif self.risk_section == "weather":
            self._choice_row(screen, x, y, "weather",
                             (("clear", "CLEAR"), ("rain", "RAIN"),
                              ("fog", "FOG"), ("snow", "SNOW")),
                             self.weather, panel.width - 24)
            self._text(screen, x, y + 48,
                       "Applies the selected precipitation/fog preset to the CARLA world.")
        elif self.risk_section == "posture":
            self._slider(screen, x, y + 20, panel.width - 36, "roll",
                         "Roll disturbance", self.posture["roll"])
            self._slider(screen, x, y + 74, panel.width - 36, "yaw",
                         "Yaw disturbance", self.posture["yaw"])
            self._slider(screen, x, y + 128, panel.width - 36, "lateral",
                         "Lateral disturbance", self.posture["lateral"])
            self._text(screen, x, y + 160,
                       "Each direction is sustained for 3 s before reversal (20 Hz PL sampling).")

        self._text(screen, x, panel.bottom - 25,
                   "These controls change the CARLA world/vehicle; resulting sensors are sent to FPGA.")

    def _choice_row(self, screen, x, y, action, choices, selected, width) -> None:
        gap = 8
        button_w = (width - gap * (len(choices) - 1)) // len(choices)
        for index, (value, label) in enumerate(choices):
            rect = pygame.Rect(x + index * (button_w + gap), y, button_w, 34)
            self._button(screen, rect, label, value == selected, action, value, small=True)

    def _slider(self, screen, x, y, width, name, label, value) -> None:
        self._text(screen, x, y - 22, f"{label}: {value}%")
        rect = pygame.Rect(x, y, width, 8)
        pygame.draw.rect(screen, (55, 75, 88), rect, border_radius=4)
        fill = pygame.Rect(rect.x, rect.y, round(rect.width * value / 100), rect.height)
        pygame.draw.rect(screen, (255, 105, 75), fill, border_radius=4)
        knob_x = rect.x + round(rect.width * value / 100)
        pygame.draw.circle(screen, (245, 250, 255), (knob_x, rect.centery), 8)
        self._sliders.append((rect, name))

    def _text(self, screen, x, y, value) -> None:
        screen.blit(self._small_font.render(value, True, (190, 215, 230)), (x, y))

    def _button(self, screen, rect, label, active, action, value, small=False) -> None:
        color = (170, 45, 45) if active else (42, 63, 78)
        border = (255, 100, 90) if active else (105, 145, 170)
        pygame.draw.rect(screen, color, rect, border_radius=5)
        pygame.draw.rect(screen, border, rect, 2, border_radius=5)
        font = self._small_font if small else self._font
        text = font.render(label, True, (245, 250, 255))
        screen.blit(text, text.get_rect(center=rect.center))
        self._buttons.append((rect, action, value))
