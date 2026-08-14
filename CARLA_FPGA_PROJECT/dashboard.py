"""First-person Mustang-style cockpit for the CARLA/FPGA demo."""

from __future__ import annotations

import math

import pygame

from control_panel import SENSORS, decode_risk_word


SENSOR_LABEL = {
    "distance": "Front Distance", "approach_speed": "Closing Speed",
    "accel_x": "Acceleration X", "accel_y": "Acceleration Y", "accel_z": "Acceleration Z",
    "gyro_x": "Angular Rate X", "gyro_y": "Angular Rate Y", "gyro_z": "Angular Rate Z",
    "temperature": "Temperature", "humidity": "Humidity", "lux": "Illuminance",
}
CHECK_LABEL = {
    "range": "Range Error", "jump": "Jump Error", "stuck": "Stuck Error",
    "noise": "Noise Error", "consistency": "Consistency Error",
    "timeout": "Communication Timeout",
}
RELIABILITY_LABEL = ("NORMAL", "DEGRADED", "INVALID", "RESERVED")
WEATHER_LABEL = {0: "CLEAR", 1: "FOG", 2: "RAIN", 3: "SNOW"}

RISK_LEVEL_LABEL = {
    "collision": ("SAFE", "CAUTION", "DANGER", "CRITICAL", "EMERGENCY"),
    "road_surface": ("DRY", "WET", "ICE", "BLACK ICE"),
    "road_impact": ("NORMAL", "ROUGH", "SEVERE", "EXTREME"),
    "visibility_light": ("BRIGHT", "DIM", "DARK", "VERY DARK"),
    "visibility_weather": ("CLEAR", "FOG", "RAIN", "SNOW"),
    "roll": ("SAFE", "ROLL DANGER"),
    "yaw": ("SAFE", "YAW CAUTION", "YAW DANGER"),
    "lateral": ("SAFE", "LATERAL CAUTION", "LATERAL DANGER"),
}
RISK_LABEL = {
    "collision": "Collision", "road_surface": "Road Surface", "road_impact": "Road Impact",
    "visibility_light": "Light Visibility", "visibility_weather": "Weather Visibility",
    "roll": "Roll Stability", "yaw": "Yaw Stability", "lateral": "Lateral Stability",
}

CREAM = (238, 224, 184)
AMBER = (236, 160, 56)
CHROME = (178, 181, 176)
GREEN = (100, 225, 150)
RED = (247, 91, 75)
BLUE = (91, 192, 255)
_FONT_CACHE = {}


def _font(size, bold=False):
    key = (int(size), bool(bold))
    if key not in _FONT_CACHE:
        _FONT_CACHE[key] = pygame.font.SysFont(
            "malgungothic", max(9, int(size)), bold=bold,
        )
    return _FONT_CACHE[key]


def _text(surface, value, position, size=15, color=(225, 232, 242),
          bold=False, center=False):
    rendered = _font(size, bold).render(str(value), True, color)
    rect = rendered.get_rect(center=position) if center else rendered.get_rect(topleft=position)
    surface.blit(rendered, rect)
    return rect


def _fit_text(surface, value, rect, size=13, color=(225, 232, 242), bold=False):
    """Draw one line, reducing only that line when a long Korean fault name needs it."""
    current = max(9, int(size))
    rendered = _font(current, bold).render(str(value), True, color)
    while current > 9:
        if rendered.get_width() <= rect.width:
            break
        current -= 1
        rendered = _font(current, bold).render(str(value), True, color)
    surface.blit(rendered, rendered.get_rect(midleft=(rect.x, rect.centery)))


def steering_wheel_angle(steering_normalized):
    """Map CARLA's normalized road-wheel command to a 900-degree hand wheel."""
    steering = max(-1.0, min(1.0, float(steering_normalized)))
    return steering * 450.0


def _reference_gauge(surface, center, radius, value, maximum, major_labels,
                     kind, gear="D1", limit=0.0, accent=AMBER):
    """Draw the twin circular gauge style used by the supplied reference."""
    cx, cy = center
    pygame.draw.circle(surface, (2, 4, 8), center, radius + 10)
    pygame.draw.circle(surface, (76, 82, 92), center, radius + 9, 3)
    pygame.draw.circle(surface, (175, 183, 194), center, radius + 5, 1)
    pygame.draw.circle(surface, (5, 9, 15), center, radius)
    pygame.draw.circle(surface, (15, 31, 54), center, radius - 8, 2)

    # Blue perimeter glow closely follows the reference cluster.
    glow_rect = pygame.Rect(cx - radius + 5, cy - radius + 5,
                            (radius - 5) * 2, (radius - 5) * 2)
    pygame.draw.arc(surface, (38, 116, 195), glow_rect,
                    math.radians(125), math.radians(415), max(2, radius // 28))

    start_deg, sweep_deg = 140.0, 260.0
    for index in range(51):
        angle = math.radians(start_deg + sweep_deg * index / 50.0)
        major = index % 5 == 0
        inner = radius - (22 if major else 14)
        outer = radius - 7
        p1 = (cx + math.cos(angle) * inner, cy + math.sin(angle) * inner)
        p2 = (cx + math.cos(angle) * outer, cy + math.sin(angle) * outer)
        pygame.draw.line(surface, (225, 232, 239) if major else (104, 121, 140), p1, p2,
                         2 if major else 1)

    for index, label in enumerate(major_labels):
        angle = math.radians(start_deg + sweep_deg * index / (len(major_labels) - 1))
        label_r = radius - 35
        point = (cx + math.cos(angle) * label_r, cy + math.sin(angle) * label_r)
        _text(surface, label, point, max(9, radius // 10),
              (226, 234, 242), center=True)

    ratio = max(0.0, min(1.0, float(value) / max(1.0, float(maximum))))
    angle = math.radians(start_deg + sweep_deg * ratio)
    needle_tip = (cx + math.cos(angle) * (radius - 26),
                  cy + math.sin(angle) * (radius - 26))
    needle_tail = (cx - math.cos(angle) * 14, cy - math.sin(angle) * 14)
    pygame.draw.line(surface, (34, 12, 10), needle_tail, needle_tip, 7)
    pygame.draw.line(surface, accent, needle_tail, needle_tip, 3)
    pygame.draw.circle(surface, (10, 13, 17), center, 11)
    pygame.draw.circle(surface, (183, 190, 199), center, 11, 2)

    # A dark center disc keeps the digital readout clear over the needle.
    pygame.draw.circle(surface, (5, 8, 13), center, max(36, radius // 3))
    pygame.draw.circle(surface, (84, 96, 110), center,
                       max(36, radius // 3), 1)
    if kind == "rpm":
        _text(surface, "GEAR", (cx, cy - radius * 0.23),
              max(9, radius // 11), (205, 215, 225), True, True)
        _text(surface, gear, (cx, cy + radius * 0.02),
              max(25, radius // 3), (247, 250, 252), True, True)
        _text(surface, f"RPM {int(value):04d}", (cx, cy + radius * 0.31),
              max(9, radius // 12), (187, 201, 216), True, True)
    else:
        _text(surface, "km/h", (cx, cy - radius * 0.23),
              max(9, radius // 11), (205, 215, 225), True, True)
        _text(surface, f"{float(value):.0f}", (cx, cy + radius * 0.02),
              max(27, radius // 3), (247, 250, 252), True, True)
        _text(surface, f"LIMIT {float(limit):.0f}",
              (cx, cy + radius * 0.31), max(9, radius // 12),
              (187, 201, 216), True, True)


def _spoke_polygon(center, inner_radius, outer_radius, angle_deg, inner_half, outer_half):
    angle = math.radians(angle_deg)
    tangent = (-math.sin(angle), math.cos(angle))
    direction = (math.cos(angle), math.sin(angle))
    inner = (center[0] + direction[0] * inner_radius,
             center[1] + direction[1] * inner_radius)
    outer = (center[0] + direction[0] * outer_radius,
             center[1] + direction[1] * outer_radius)
    return [
        (inner[0] + tangent[0] * inner_half, inner[1] + tangent[1] * inner_half),
        (outer[0] + tangent[0] * outer_half, outer[1] + tangent[1] * outer_half),
        (outer[0] - tangent[0] * outer_half, outer[1] - tangent[1] * outer_half),
        (inner[0] - tangent[0] * inner_half, inner[1] - tangent[1] * inner_half),
    ]


def _build_mustang_wheel(radius):
    """Build a rotatable 3-spoke wood/chrome wheel inspired by a 1960s Mustang."""
    pad = max(8, radius // 12)
    size = (radius + pad) * 2
    wheel = pygame.Surface((size, size), pygame.SRCALPHA)
    center = (size // 2, size // 2)

    # Deep wood rim, a thin chrome inner ring, and warm varnish highlight.
    pygame.draw.circle(wheel, (31, 14, 8), center, radius, max(12, radius // 8))
    pygame.draw.circle(wheel, (106, 47, 20), center, radius, max(9, radius // 12))
    pygame.draw.circle(wheel, (185, 104, 48), center, radius, max(3, radius // 42))
    pygame.draw.arc(wheel, (239, 166, 92),
                    pygame.Rect(center[0] - radius, center[1] - radius,
                                radius * 2, radius * 2),
                    math.radians(205), math.radians(335), max(2, radius // 65))
    pygame.draw.circle(wheel, (170, 171, 164), center, radius - radius // 10, 2)

    hub_radius = max(31, radius // 4)
    for spoke_angle in (-90, 30, 150):
        polygon = _spoke_polygon(
            center, hub_radius - 4, radius - radius // 8, spoke_angle,
            radius * 0.12, radius * 0.065,
        )
        pygame.draw.polygon(wheel, (151, 153, 148), polygon)
        pygame.draw.lines(wheel, (226, 224, 211), True, polygon, 2)
        # First-generation sport wheels used drilled metal spokes.
        angle = math.radians(spoke_angle)
        for fraction in (0.43, 0.59, 0.74):
            distance = hub_radius + (radius - hub_radius) * fraction
            hole = (round(center[0] + math.cos(angle) * distance),
                    round(center[1] + math.sin(angle) * distance))
            pygame.draw.circle(wheel, (20, 20, 18), hole, max(3, radius // 27))
            pygame.draw.circle(wheel, (91, 91, 85), hole, max(3, radius // 27), 1)

    pygame.draw.circle(wheel, (30, 27, 22), center, hub_radius + 7)
    pygame.draw.circle(wheel, (186, 176, 147), center, hub_radius + 7, 3)
    pygame.draw.circle(wheel, (100, 43, 20), center, hub_radius - 3)
    pygame.draw.circle(wheel, (225, 197, 130), center, hub_radius - 3, 2)

    # A small, readable pony-like emblem gives the hub a period-correct identity.
    horse = (238, 216, 161)
    emblem_scale = max(1, radius / 180.0)
    points = [(-27, 2), (-12, -6), (5, -8), (15, -17), (25, -15),
              (17, -7), (30, -2), (14, 1), (6, 10), (-10, 8), (-22, 14)]
    points = [(center[0] + x * emblem_scale, center[1] + y * emblem_scale)
              for x, y in points]
    pygame.draw.lines(wheel, horse, False, points, max(2, round(3 * emblem_scale)))
    return wheel


def _reliability_lines(control_panel, fpga_result):
    active = sorted(control_panel.injector.sensor_faults)
    if fpga_result is None:
        if active:
            return [(f"{SENSOR_LABEL[sensor]} · {CHECK_LABEL[check]} · WAITING FOR PL",
                     AMBER) for sensor, check in active[:2]]
        return [("NO FPGA RESPONSE", AMBER)]

    word = int(fpga_result.reliability_word)
    lines = []
    named = set()
    sensor_index = {name: index for index, (name, _label) in enumerate(SENSORS)}
    for sensor_name, check in active:
        state = (word >> (2 * sensor_index[sensor_name])) & 0x3
        lines.append((f"{SENSOR_LABEL[sensor_name]} · {CHECK_LABEL[check]} · {RELIABILITY_LABEL[state]}",
                      RED if state else AMBER))
        named.add(sensor_name)
    for index, (sensor_name, _label) in enumerate(SENSORS):
        state = (word >> (2 * index)) & 0x3
        if state and sensor_name not in named:
            lines.append((f"{SENSOR_LABEL[sensor_name]} · {RELIABILITY_LABEL[state]}", RED))
    return lines[:2] or [("ALL SENSORS NORMAL", GREEN)]


def _selected_risk_lines(control_panel):
    selected = []
    if control_panel.collision_tier:
        selected.append(
            ("SAFE", "COLLISION CAUTION", "COLLISION DANGER",
             "COLLISION CRITICAL", "COLLISION EMERGENCY")[control_panel.collision_tier]
        )
    elif control_panel.collision_active:
        selected.append("FRONT OBSTACLE")
    if control_panel.road_surface != "dry":
        selected.append({"wet": "WET ROAD", "ice": "ICE", "black_ice": "BLACK ICE"}[
            control_panel.road_surface])
    if control_panel.roughness:
        selected.append(f"ROAD IMPACT {control_panel.roughness}%")
    if control_panel.visibility_risk:
        selected.append(f"VISIBILITY LOSS {control_panel.visibility_risk}%")
    if control_panel.weather != "clear":
        selected.append({"rain": "RAIN", "fog": "FOG", "snow": "SNOW"}[control_panel.weather])
    for name, label in (("roll", "ROLL"), ("yaw", "YAW"), ("lateral", "LATERAL")):
        if control_panel.posture[name]:
            selected.append(f"{label} POSTURE {control_panel.posture[name]}%")
    return selected


def _risk_lines(fpga_result, control_panel):
    selected = _selected_risk_lines(control_panel)
    if fpga_result is None:
        return ([(f"TEST CONDITION · {value} · WAITING FOR PL", AMBER) for value in selected[:2]]
                if selected else [("NO RISK DATA", (160, 178, 200))])
    risks = decode_risk_word(fpga_result.risk_word)
    lines = []
    for name, level in risks.items():
        if not level:
            continue
        names = RISK_LEVEL_LABEL[name]
        state = names[level] if level < len(names) else f"LEVEL {level}"
        lines.append((f"{RISK_LABEL[name]} · {state}", (255, 155, 75)))
    if lines:
        return lines[:2]
    if selected:
        return [(f"TEST CONDITION · {value} · WAITING FOR PL", AMBER) for value in selected[:2]]
    return [("NO RISK DETECTED", GREEN)]


def _fpga_status(control_panel, fpga_result, actuation_active):
    if not control_panel.apply_fpga_output:
        return "FPGA BYPASS · CARLA NORMAL DRIVE", (155, 175, 198)
    if not control_panel.intervention_scenario_active:
        return "FPGA ARMED · CARLA NORMAL DRIVE", BLUE
    if fpga_result is None:
        return "TEST ACTIVE · WAITING FOR FPGA", AMBER
    if actuation_active:
        return "FPGA INTERVENTION · SAFETY CONTROL ACTIVE", RED
    return "FPGA ARMED", BLUE


def _draw_left_cockpit(screen, sensor, environment, controller, vehicle_control, scale):
    screen_h = screen.get_height()
    panel_w = min(round(760 * scale), round(screen.get_width() * 0.62))
    panel_h = round(266 * scale)
    panel = pygame.Surface((panel_w, panel_h), pygame.SRCALPHA)

    # Black vinyl dash with a slim walnut fascia and bright-metal trim.
    pygame.draw.rect(panel, (10, 9, 8, 224), panel.get_rect(), border_radius=round(32 * scale))
    pygame.draw.rect(panel, (78, 35, 18, 235),
                     (round(10 * scale), round(13 * scale), panel_w - round(20 * scale),
                      panel_h - round(25 * scale)), border_radius=round(27 * scale))
    for stripe in range(5):
        y = round((30 + stripe * 43) * scale)
        pygame.draw.line(panel, (104 + stripe * 3, 50, 23, 65),
                         (round(22 * scale), y), (panel_w - round(22 * scale), y), 1)
    pygame.draw.rect(panel, (9, 10, 9, 245),
                     (round(18 * scale), round(21 * scale), panel_w - round(36 * scale),
                      panel_h - round(42 * scale)), border_radius=round(22 * scale))
    pygame.draw.rect(panel, CHROME,
                     (round(18 * scale), round(21 * scale), panel_w - round(36 * scale),
                      panel_h - round(42 * scale)), max(1, round(2 * scale)),
                     border_radius=round(22 * scale))

    radius = round(91 * scale)
    gauge_y = round(126 * scale)
    rpm_x = round(124 * scale)
    speed_x = round(326 * scale)
    rpm = max(0, int(controller.current_rpm))
    _classic_gauge(panel, (rpm_x, gauge_y), radius, rpm, 7000, "RPM ×1000",
                   f"{rpm / 1000:.1f}", tuple(range(8)), (237, 116, 69))
    _classic_gauge(panel, (speed_x, gauge_y), radius, sensor.speed, 200, "km/h",
                   f"{sensor.speed:03.0f}", tuple(range(0, 201, 20)), (241, 188, 69))

    gear = "R" if vehicle_control.reverse else f"D{controller.current_gear + 1}"
    _text(panel, f"{gear}   LIMIT {environment.speed_limit:.0f}",
          (round(225 * scale), round(238 * scale)), round(13 * scale), CREAM, True, True)
    panel_top = screen_h - panel_h - round(7 * scale)
    screen.blit(panel, (round(8 * scale), panel_top))

    # The wheel is deliberately large and partly below the screen, matching a
    # first-person driving-game camera instead of a small HUD icon.
    wheel_radius = round(176 * scale)
    wheel = _build_mustang_wheel(wheel_radius)
    angle = steering_wheel_angle(vehicle_control.steer)
    # CARLA positive steering is right; pygame positive rotation is CCW.
    rotated = pygame.transform.rotozoom(wheel, -angle, 1.0)
    wheel_center = (round(600 * scale), screen_h + round(34 * scale))
    screen.blit(rotated, rotated.get_rect(center=wheel_center))


def _draw_right_information(screen, sensor, environment, controller, command,
                            vehicle_control, fpga_result, control_panel,
                            actuation_active, scale):
    margin = round(12 * scale)
    button_reserve = round(52 * scale)
    panel_w = min(round(500 * scale), round(screen.get_width() * 0.39))
    panel_h = min(round(342 * scale), screen.get_height() - button_reserve - margin * 2)
    left = screen.get_width() - panel_w - margin
    top = screen.get_height() - panel_h - button_reserve
    panel = pygame.Surface((panel_w, panel_h), pygame.SRCALPHA)

    pygame.draw.rect(panel, (7, 10, 12, 225), panel.get_rect(),
                     border_radius=round(16 * scale))
    pygame.draw.rect(panel, (117, 92, 52, 245), panel.get_rect(),
                     max(1, round(2 * scale)), border_radius=round(16 * scale))
    pygame.draw.rect(panel, (33, 31, 26, 220),
                     (round(8 * scale), round(8 * scale), panel_w - round(16 * scale),
                      panel_h - round(16 * scale)), max(1, round(scale)),
                     border_radius=round(12 * scale))

    fs = max(10, round(12 * scale))
    small = max(9, round(11 * scale))
    line_h = max(15, round(18 * scale))
    pad = round(15 * scale)
    half = (panel_w - pad * 3) // 2
    x1, x2 = pad, pad * 2 + half

    _text(panel, "MUSTANG · FPGA DRIVE", (pad, round(12 * scale)), fs + 2, CREAM, True)
    status, status_color = _fpga_status(control_panel, fpga_result, actuation_active)
    _fit_text(panel, status,
              pygame.Rect(x2, round(10 * scale), half, line_h), fs, status_color, True)
    separator_y = round(37 * scale)
    pygame.draw.line(panel, (100, 78, 45), (pad, separator_y),
                     (panel_w - pad, separator_y), 1)

    steer_angle = steering_wheel_angle(vehicle_control.steer)
    mode = "MANUAL" if command.manual_mode else "AUTO"
    gear = "R" if vehicle_control.reverse else f"D{controller.current_gear + 1}"
    left_lines = (
        ("VEHICLE INPUT", AMBER, True),
        (f"Mode {mode}  ·  Gear {gear}", CREAM, False),
        (f"Front {sensor.distance:6.1f} m", (216, 226, 232), False),
        (f"Closing {sensor.approach_speed:+6.2f} m/s", (216, 226, 232), False),
        (f"Accel X/Y/Z", (154, 174, 185), False),
        (f"{sensor.accel_x:+.2f} / {sensor.accel_y:+.2f} / {sensor.accel_z:+.2f}", CREAM, False),
        (f"Gyro X/Y/Z", (154, 174, 185), False),
        (f"{sensor.gyro_x:+.2f} / {sensor.gyro_y:+.2f} / {sensor.gyro_z:+.2f}", CREAM, False),
        (f"Incline X/Y/Z", (154, 174, 185), False),
        (f"{sensor.incline_x:+.1f} / {sensor.incline_y:+.1f} / {sensor.incline_z:+.1f}", CREAM, False),
    )
    right_lines = (
        ("ENVIRONMENT · CONTROL", AMBER, True),
        (f"Weather {WEATHER_LABEL.get(environment.weather, environment.weather)}  ·  Limit {environment.speed_limit:.0f}", CREAM, False),
        (f"Temp {sensor.temperature:.1f}°C  ·  Humidity {sensor.humidity:.0f}%", (216, 226, 232), False),
        (f"Illuminance {sensor.lux:.0f} lux", (216, 226, 232), False),
        (f"Throttle {command.throttle:2d}/10  ·  Brake {command.brake:2d}/10", CREAM, False),
        (f"Hand Wheel {steer_angle:+.0f}° / ±450°", CREAM, False),
        (f"Steering Command {vehicle_control.steer:+.2f}", (216, 226, 232), False),
        (f"Steering Limit {command.steering_rate_limit:3d}%", (216, 226, 232), False),
        (f"Sample #{getattr(fpga_result, 'sample_seq', '-')}", (154, 174, 185), False),
        ("C key · First/Third Person", (154, 174, 185), False),
    )
    start_y = separator_y + round(8 * scale)
    for index, (value, color, bold) in enumerate(left_lines):
        _fit_text(panel, value,
                  pygame.Rect(x1, start_y + index * line_h, half, line_h),
                  small, color, bold)
    for index, (value, color, bold) in enumerate(right_lines):
        _fit_text(panel, value,
                  pygame.Rect(x2, start_y + index * line_h, half, line_h),
                  small, color, bold)

    diag_y = start_y + len(left_lines) * line_h + round(5 * scale)
    pygame.draw.line(panel, (100, 78, 45), (pad, diag_y),
                     (panel_w - pad, diag_y), 1)
    diag_y += round(6 * scale)
    rel = _reliability_lines(control_panel, fpga_result)
    risks = _risk_lines(fpga_result, control_panel)
    _text(panel, "RELIABILITY", (x1, diag_y), small, BLUE, True)
    _text(panel, "RISK", (x2, diag_y), small, (255, 175, 77), True)
    for index, (value, color) in enumerate(rel):
        row_y = diag_y + (index + 1) * line_h
        _fit_text(panel, value, pygame.Rect(x1, row_y, half, line_h), small, color)
    for index, (value, color) in enumerate(risks):
        row_y = diag_y + (index + 1) * line_h
        _fit_text(panel, value, pygame.Rect(x2, row_y, half, line_h), small, color)

    screen.blit(panel, (left, top))


def draw_dashboard(screen, sensor, environment, controller, command,
                   vehicle_control, fpga_result, control_panel,
                   actuation_active):
    """Draw the first-person cockpit without obscuring the upper driving view."""
    scale = max(0.72, min(1.35, min(screen.get_width() / 1280.0,
                                    screen.get_height() / 720.0)))
    _draw_left_cockpit(screen, sensor, environment, controller, vehicle_control, scale)
    _draw_right_information(
        screen, sensor, environment, controller, command, vehicle_control,
        fpga_result, control_panel, actuation_active, scale,
    )


# ---------------------------------------------------------------------------
# Reference-cluster HUD (the definition below intentionally replaces the
# earlier dashboard entry point while retaining the reusable wheel helpers).
# ---------------------------------------------------------------------------

def _clean_reliability_lines(control_panel, fpga_result):
    active = sorted(control_panel.injector.sensor_faults)
    if fpga_result is None:
        if active:
            return [
                (f"{SENSOR_LABEL[sensor]} | {CHECK_LABEL[check]} | WAITING FOR PL", AMBER)
                for sensor, check in active[:2]
            ]
        return [("RELIABILITY | NO FPGA RESPONSE", AMBER)]

    word = int(fpga_result.reliability_word)
    sensor_index = {name: index for index, (name, _label) in enumerate(SENSORS)}
    lines = []
    named = set()
    for sensor_name, check in active:
        state = (word >> (2 * sensor_index[sensor_name])) & 0x3
        color = GREEN if state == 0 else (AMBER if state == 1 else RED)
        lines.append((
            f"{SENSOR_LABEL[sensor_name]} | {CHECK_LABEL[check]} | "
            f"{RELIABILITY_LABEL[state]}", color,
        ))
        named.add(sensor_name)
    for index, (sensor_name, _label) in enumerate(SENSORS):
        state = (word >> (2 * index)) & 0x3
        if state and sensor_name not in named:
            color = AMBER if state == 1 else RED
            lines.append((
                f"{SENSOR_LABEL[sensor_name]} | {RELIABILITY_LABEL[state]}", color,
            ))
    return lines[:2] or [("ALL SENSORS NORMAL", GREEN)]


def _clean_risk_lines(fpga_result, control_panel):
    selected = _selected_risk_lines(control_panel)
    if fpga_result is None:
        if selected:
            return [(f"{value} | WAITING FOR PL", AMBER) for value in selected[:2]]
        return [("NO RISK DATA", (180, 194, 210))]

    risks = decode_risk_word(fpga_result.risk_word)
    lines = []
    for name, level in risks.items():
        if not level:
            continue
        names = RISK_LEVEL_LABEL[name]
        state = names[level] if level < len(names) else f"LEVEL {level}"
        lines.append((f"{RISK_LABEL[name]} | {state}", (255, 164, 72)))
    if lines:
        return lines[:2]
    if selected:
        return [(f"{value} | WAITING FOR PL", AMBER) for value in selected[:2]]
    return [("NO ACTIVE RISK", GREEN)]


def _floating_text(screen, value, position, size, color, bold=False,
                   center=False):
    """Readable text with a shadow, but no opaque HUD background."""
    shadow_pos = (position[0] + 2, position[1] + 2)
    _text(screen, value, shadow_pos, size, (0, 0, 0), bold, center)
    return _text(screen, value, position, size, color, bold, center)


def _draw_reference_cluster(screen, sensor, environment, controller,
                            vehicle_control, scale):
    screen_w, screen_h = screen.get_size()
    cluster_w = min(round(880 * scale), round(screen_w * 0.72))
    cluster_h = min(round(242 * scale), round(screen_h * 0.35))
    left = (screen_w - cluster_w) // 2
    top = screen_h - cluster_h - max(3, round(4 * scale))
    cluster = pygame.Surface((cluster_w, cluster_h), pygame.SRCALPHA)

    outer = cluster.get_rect()
    radius_border = max(24, round(42 * scale))
    pygame.draw.rect(cluster, (5, 7, 13, 238), outer,
                     border_radius=radius_border)
    pygame.draw.rect(cluster, (72, 72, 84, 245), outer,
                     max(3, round(4 * scale)), border_radius=radius_border)
    pygame.draw.rect(cluster, (176, 167, 144, 210),
                     outer.inflate(-round(11 * scale), -round(11 * scale)),
                     max(1, round(2 * scale)), border_radius=radius_border)

    # Violet/black upper brow and a narrow lower information rail mirror the
    # supplied production cluster without copying an image into the project.
    brow = pygame.Rect(round(16 * scale), round(12 * scale),
                       cluster_w - round(32 * scale), round(42 * scale))
    pygame.draw.rect(cluster, (20, 13, 31, 230), brow,
                     border_radius=round(18 * scale))
    rail_y = cluster_h - round(35 * scale)
    pygame.draw.line(cluster, (82, 91, 105),
                     (round(40 * scale), rail_y),
                     (cluster_w - round(40 * scale), rail_y), 1)

    gauge_radius = min(round(88 * scale), (cluster_h - round(28 * scale)) // 2)
    gauge_y = round(119 * scale)
    gauge_offset = round(156 * scale)
    rpm_center = (gauge_offset, gauge_y)
    speed_center = (cluster_w - gauge_offset, gauge_y)
    rpm = max(0, int(controller.current_rpm))
    gear = "R" if vehicle_control.reverse else f"D{controller.current_gear + 1}"

    _reference_gauge(
        cluster, rpm_center, gauge_radius, rpm, 7000, tuple(range(8)),
        "rpm", gear=gear, accent=(239, 82, 74),
    )
    _reference_gauge(
        cluster, speed_center, gauge_radius, sensor.speed, 200,
        tuple(range(0, 201, 20)), "speed", limit=environment.speed_limit,
        accent=(92, 177, 255),
    )

    # Small center auxiliary gauge from the reference cluster.
    aux_center = (cluster_w // 2, round(80 * scale))
    aux_r = round(29 * scale)
    pygame.draw.circle(cluster, (4, 7, 12), aux_center, aux_r + 5)
    pygame.draw.circle(cluster, (112, 125, 141), aux_center, aux_r + 4, 2)
    pygame.draw.circle(cluster, (23, 43, 67), aux_center, aux_r, 2)
    _text(cluster, "TEMP", (aux_center[0], aux_center[1] - round(9 * scale)),
          max(8, round(8 * scale)), (192, 205, 218), True, True)
    _text(cluster, f"{sensor.temperature:.0f} C",
          (aux_center[0], aux_center[1] + round(8 * scale)),
          max(9, round(9 * scale)), (238, 243, 247), True, True)

    # PRNDS rail; the active gear is highlighted in the same location as the
    # reference photo, while the large current gear remains inside the RPM dial.
    gear_index = 1 if vehicle_control.reverse else 3
    gear_chars = "PRNDS"
    gear_x = cluster_w // 2 - round(43 * scale)
    for index, label in enumerate(gear_chars):
        color = AMBER if index == gear_index else (144, 153, 164)
        _text(cluster, label, (gear_x + index * round(18 * scale), rail_y + round(7 * scale)),
              max(9, round(10 * scale)), color, True)

    screen.blit(cluster, (left, top))

    # The first-generation Mustang wheel is centered exactly between the RPM
    # and speed gauges and rotates through the full -450..+450 degree range.
    wheel_radius = round(166 * scale)
    wheel = _build_mustang_wheel(wheel_radius)
    angle = steering_wheel_angle(vehicle_control.steer)
    rotated = pygame.transform.rotozoom(wheel, -angle, 1.0)
    wheel_center = (screen_w // 2, screen_h + round(30 * scale))
    screen.blit(rotated, rotated.get_rect(center=wheel_center))
    return pygame.Rect(left, top, cluster_w, cluster_h)


def _draw_status_overlay(screen, cluster_rect, fpga_result, control_panel,
                         scale):
    font_size = max(12, round(12 * scale))
    header_size = max(11, round(11 * scale))
    left_x = cluster_rect.left + round(10 * scale)
    right_x = cluster_rect.centerx + round(65 * scale)
    base_y = cluster_rect.top - round(61 * scale)

    _floating_text(screen, "RELIABILITY", (left_x, base_y),
                   header_size, BLUE, True)
    for index, (value, color) in enumerate(
            _clean_reliability_lines(control_panel, fpga_result)):
        _floating_text(screen, value,
                       (left_x, base_y + round((17 + index * 18) * scale)),
                       font_size, color, True)

    _floating_text(screen, "RISK", (right_x, base_y),
                   header_size, (255, 174, 75), True)
    for index, (value, color) in enumerate(
            _clean_risk_lines(fpga_result, control_panel)):
        _floating_text(screen, value,
                       (right_x, base_y + round((17 + index * 18) * scale)),
                       font_size, color, True)

    if fpga_result is None:
        return

    warning_y = cluster_rect.top - round(111 * scale)
    if fpga_result.hud_warning:
        _floating_text(screen, "DRIVER SAFETY WARNING",
                       (cluster_rect.centerx, warning_y),
                       max(15, round(16 * scale)), RED, True, True)
        warning_y += round(22 * scale)
    if fpga_result.transition_demand:
        _floating_text(screen, "Manual Mode로 전환하시길 바랍니다",
                       (cluster_rect.centerx, warning_y),
                       max(14, round(15 * scale)), AMBER, True, True)
        # The remaining-time field is intentionally displayed as digits only.
        _floating_text(screen, str(int(fpga_result.td_remain_sec)),
                       (cluster_rect.centerx + round(250 * scale), warning_y),
                       max(22, round(24 * scale)), AMBER, True, True)
        warning_y += round(24 * scale)
    if fpga_result.mrm:
        _floating_text(screen, "MINIMUM RISK MANEUVER ACTIVE",
                       (cluster_rect.centerx, warning_y),
                       max(14, round(15 * scale)), RED, True, True)


def _reliability_state(fpga_result, sensor_name):
    if fpga_result is None:
        return "NO DATA", (142, 154, 169)
    index = next(i for i, item in enumerate(SENSORS) if item[0] == sensor_name)
    state = (int(fpga_result.reliability_word) >> (2 * index)) & 0x3
    color = GREEN if state == 0 else (AMBER if state == 1 else RED)
    return RELIABILITY_LABEL[state], color


def _draw_fpga_input_monitor(screen, sensor, environment, fpga_result,
                             control_panel, snapshot, input_words, scale):
    button_w = min(210, round(166 * scale))
    button_h = max(31, round(29 * scale))
    margin = max(10, round(12 * scale))
    button = pygame.Rect(screen.get_width() - button_w - margin,
                         margin, button_w, button_h)
    control_panel.fpga_input_toggle_rect = button.copy()

    active = bool(control_panel.show_fpga_inputs)
    button_bg = (28, 113, 167, 225) if active else (7, 13, 20, 205)
    layer = pygame.Surface(button.size, pygame.SRCALPHA)
    pygame.draw.rect(layer, button_bg, layer.get_rect(), border_radius=7)
    pygame.draw.rect(layer, (93, 198, 255, 245), layer.get_rect(), 2,
                     border_radius=7)
    screen.blit(layer, button.topleft)
    _text(screen, "FPGA INPUTS", button.center,
          max(12, round(12 * scale)), (235, 246, 252), True, True)
    if not active:
        return

    panel_w = min(560, screen.get_width() - margin * 2)
    panel_h = min(590, screen.get_height() - button.bottom - margin * 2)
    panel_rect = pygame.Rect(screen.get_width() - panel_w - margin,
                             button.bottom + 7, panel_w, panel_h)
    panel = pygame.Surface(panel_rect.size, pygame.SRCALPHA)
    pygame.draw.rect(panel, (3, 8, 14, 224), panel.get_rect(), border_radius=11)
    pygame.draw.rect(panel, (72, 174, 229, 238), panel.get_rect(), 2,
                     border_radius=11)

    pad = 13
    y = 10
    fs = max(11, min(14, panel_h // 42))
    row_h = max(18, fs + 5)
    sample_seq = snapshot.get("sample_seq", "-") if snapshot else "-"
    _text(panel, f"FPGA INPUT SNAPSHOT  |  SAMPLE #{sample_seq}",
          (pad, y), fs + 2, BLUE, True)
    y += row_h + 4
    _text(panel, "CHANNEL", (pad, y), fs, (145, 163, 181), True)
    value_x = round(panel_w * 0.43)
    result_x = round(panel_w * 0.72)
    _text(panel, "INPUT TO PL", (value_x, y), fs, (145, 163, 181), True)
    _text(panel, "PL RELIABILITY", (result_x, y), fs, (145, 163, 181), True)
    y += row_h
    pygame.draw.line(panel, (45, 75, 96), (pad, y - 2), (panel_w - pad, y - 2), 1)

    rows = (
        ("Front Distance", f"{sensor.distance:8.2f} m", "distance"),
        ("Closing Speed", f"{sensor.approach_speed:+8.2f} m/s", "approach_speed"),
        ("Acceleration X", f"{sensor.accel_x:+8.3f} m/s2", "accel_x"),
        ("Acceleration Y", f"{sensor.accel_y:+8.3f} m/s2", "accel_y"),
        ("Acceleration Z", f"{sensor.accel_z:+8.3f} m/s2", "accel_z"),
        ("Angular Rate X", f"{sensor.gyro_x:+8.4f} rad/s", "gyro_x"),
        ("Angular Rate Y", f"{sensor.gyro_y:+8.4f} rad/s", "gyro_y"),
        ("Angular Rate Z", f"{sensor.gyro_z:+8.4f} rad/s", "gyro_z"),
        ("Temperature", f"{sensor.temperature:+8.1f} C", "temperature"),
        ("Humidity", f"{sensor.humidity:8.1f} %", "humidity"),
        ("Illuminance", f"{sensor.lux:8.0f} lux", "lux"),
    )
    active_faults = {}
    for sensor_name, check in sorted(control_panel.injector.sensor_faults):
        active_faults.setdefault(sensor_name, []).append(CHECK_LABEL[check])
    for label, value, sensor_name in rows:
        state, state_color = _reliability_state(fpga_result, sensor_name)
        checks = active_faults.get(sensor_name)
        if checks:
            state = f"{state} ({'/'.join(checks)})"
        _fit_text(panel, label, pygame.Rect(pad, y, value_x - pad - 5, row_h),
                  fs, (214, 224, 233))
        _fit_text(panel, value, pygame.Rect(value_x, y, result_x - value_x - 5, row_h),
                  fs, CREAM)
        _fit_text(panel, state, pygame.Rect(result_x, y, panel_w - result_x - pad, row_h),
                  fs, state_color, True)
        y += row_h

    pygame.draw.line(panel, (45, 75, 96), (pad, y + 1), (panel_w - pad, y + 1), 1)
    y += 7
    _text(panel, "REFERENCE / VEHICLE INPUTS", (pad, y), fs, AMBER, True)
    y += row_h
    if snapshot:
        left_lines = (
            f"Speed XYZ  {sensor.speed_x:+.2f} / {sensor.speed_y:+.2f} / {sensor.speed_z:+.2f} m/s",
            f"Incline XYZ  {sensor.incline_x:+.1f} / {sensor.incline_y:+.1f} / {sensor.incline_z:+.1f} deg",
            f"Weather {WEATHER_LABEL.get(environment.weather, environment.weather)} | Limit {snapshot['speed_limit']:.1f} km/h | RPM level {snapshot['rpm_level']}",
            f"Throttle {snapshot['accelerator']}/10 | Brake {snapshot['brake']}/10 | Steering {snapshot['steering']:+.2f}",
            f"Mode {'MANUAL' if snapshot['manual_mode'] else 'AUTO'} | Gear {snapshot['gear']} | Situation {snapshot['situation']:03b}",
            f"Headlight {int(snapshot['headlight'])} | Hazard {int(snapshot['hazard'])}",
        )
        for value in left_lines:
            _fit_text(panel, value, pygame.Rect(pad, y, panel_w - pad * 2, row_h),
                      fs, (205, 216, 227))
            y += row_h

    if input_words and y + row_h * 4 < panel_h:
        y += 2
        _text(panel, "PACKED AXI WORDS", (pad, y), fs, BLUE, True)
        y += row_h
        for row in range(5):
            left_index, right_index = row, row + 5
            value = (
                f"REG{left_index} 0x{int(input_words[left_index]):08X}"
                f"      REG{right_index} 0x{int(input_words[right_index]):08X}"
            )
            _fit_text(panel, value, pygame.Rect(pad, y, panel_w - pad * 2, row_h),
                      fs, (159, 201, 225))
            y += row_h

    screen.blit(panel, panel_rect.topleft)


def draw_dashboard(screen, sensor, environment, controller, command,
                   vehicle_control, fpga_result, control_panel,
                   actuation_active, fpga_input_snapshot=None,
                   input_words=None):
    """Draw the reference-style cockpit and optional FPGA input monitor."""
    scale = max(0.72, min(1.35, min(screen.get_width() / 1280.0,
                                    screen.get_height() / 720.0)))
    cluster_rect = _draw_reference_cluster(
        screen, sensor, environment, controller, vehicle_control, scale,
    )
    _draw_status_overlay(
        screen, cluster_rect, fpga_result, control_panel, scale,
    )
    _draw_fpga_input_monitor(
        screen, sensor, environment, fpga_result, control_panel,
        fpga_input_snapshot or {}, input_words or (), scale,
    )
