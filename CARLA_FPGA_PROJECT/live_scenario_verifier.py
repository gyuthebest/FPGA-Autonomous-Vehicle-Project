"""Run Control Panel scenarios through live CARLA -> FPGA -> CARLA I/O."""

from __future__ import annotations

import csv
from datetime import datetime
import os
from pathlib import Path

import pygame

from control_panel import (
    CONSISTENCY_SENSORS,
    RANGE_SENSORS,
    SENSORS,
    decode_risk_word,
)


RELIABILITY_INDEX = {
    "distance": 0, "approach_speed": 1,
    "accel_x": 2, "accel_y": 3, "accel_z": 4,
    "gyro_x": 5, "gyro_y": 6, "gyro_z": 7,
    "temperature": 8, "humidity": 9, "lux": 10,
}


def _risk(name, section, tier, seconds=1.2, **control):
    return {
        "name": name, "seconds": seconds, "section": section,
        "settings": {section: tier}, "risk": (section, tier),
        "control": control,
    }


class LiveScenarioVerifier:
    """State sequencer used only when CARLA_LIVE_VERIFY=1."""

    def __init__(self, panel, world_scenarios):
        self.enabled = os.getenv("CARLA_LIVE_VERIFY", "0").lower() in {
            "1", "true", "yes", "on"
        }
        self.panel = panel
        self.world_scenarios = world_scenarios
        self.index = -1
        self.elapsed = 0.0
        self.observations = []
        self.results = []
        self.capture_due = False
        self.finished = False
        self.current = None
        self.resetting = False
        self.reset_elapsed = 0.0
        self.force_manual_mode = False
        self.current_captured = False
        self.output_dir = None
        if not self.enabled:
            return
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_dir = Path(__file__).resolve().parent / "logs" / "live_scenarios" / stamp
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.cases = self._cases()
        requested = {
            name.strip() for name in os.getenv("CARLA_LIVE_VERIFY_CASES", "").split(",")
            if name.strip()
        }
        if requested:
            self.cases = [case for case in self.cases if case["name"] in requested]
        self._advance()

    @staticmethod
    def _cases():
        # Existing RTL unit/full regressions are not repeated here.  These are
        # E2E checks that prove each Control Panel condition reaches the board
        # and returns the corresponding visible vehicle command.
        cases = [
            {"name": "baseline", "seconds": 1.2},
            _risk("collision_caution", "collision", 1, accelerator_max=0),
            _risk("collision_danger", "collision", 2, accelerator_max=0, brake_min=2),
            _risk("collision_critical", "collision", 3, accelerator_max=0, brake_min=4, hazard=1),
            _risk("collision_emergency", "collision", 4, accelerator_max=0, brake_min=10, hazard=1),
            _risk("road_wet", "road_surface", 1, speed_ratio_max=0.91, accelerator_max=8),
            _risk("road_ice", "road_surface", 2, speed_ratio_max=0.71, accelerator_max=6, brake_max=0),
            _risk("road_black_ice", "road_surface", 3, speed_ratio_max=0.51, accelerator_max=4, brake_max=0),
            # The 0.5 g rough-road measurement also makes accel_z DEGRADED
            # against the dynamics reference.  risk_control.sv therefore
            # raises the effective PL tier from ROUGH(1) to SEVERE(2).
            dict(
                _risk("impact_rough", "road_impact", 2, seconds=1.2,
                      speed_ratio_max=0.61, brake_min=2),
                settings={"road_impact": 1}, min_speed_mps=12.0,
            ),
            dict(_risk("impact_severe", "road_impact", 2, seconds=1.2, speed_ratio_max=0.61, brake_min=2), min_speed_mps=12.0),
            dict(_risk("impact_extreme", "road_impact", 3, seconds=1.2, speed_ratio_max=0.51, brake_min=2), min_speed_mps=12.0),
            _risk("visibility_dim", "visibility_light", 1, headlight=1),
            _risk("visibility_dark", "visibility_light", 2, headlight=1),
            _risk("visibility_very_dark", "visibility_light", 3, headlight=1, speed_ratio_max=0.91),
            _risk("weather_fog", "visibility_weather", 1, accelerator_max=8, headlight=1, speed_ratio_max=0.91),
            _risk("weather_rain", "visibility_weather", 2, accelerator_max=8, headlight=1, hazard=1, speed_ratio_max=0.71),
            _risk("weather_snow", "visibility_weather", 3, accelerator_max=5, headlight=1, speed_ratio_max=0.61),
            _risk("posture_roll", "roll", 1, seconds=3.2, accelerator_max=0),
            _risk("posture_yaw_caution", "yaw", 1, seconds=3.2, accelerator_max=8),
            _risk("posture_yaw_danger", "yaw", 2, seconds=3.2, accelerator_max=0),
            _risk("posture_lateral_caution", "lateral", 1, seconds=3.2, accelerator_max=7),
            _risk("posture_lateral_danger", "lateral", 2, seconds=3.2, accelerator_max=0, brake_max=0),
            {
                "name": "reliability_degraded_escalates_surface", "seconds": 1.5,
                "section": "road_surface", "settings": {"road_surface": 1},
                "fault": ("temperature", "jump"),
                "reliability": ("temperature", 1), "risk_min": ("road_surface", 2),
            },
            # 2026-08-15 정책: INVALID 는 위험도를 올리지 않는다.
            # risk_control 은 원시 tier 를 그대로 통과시키고, INVALID 대응은
            # TD/MRM 이 담당한다.  그래서 예전의 risk_min(바닥값) 기대는
            # 더 이상 성립하지 않는다.  남은 관측 가능한 효과는 두 가지다.
            #   - 해당 채널이 INVALID 로 확정된다
            #   - HUD 경고가 뜬다 (hud_warning 은 9개 그룹의 INVALID 를 OR 한다)
            {
                "name": "reliability_invalid_surface_no_escalation", "seconds": 1.5,
                "fault": ("temperature", "range"),
                "reliability": ("temperature", 2), "hud": 1,
            },
            {
                "name": "reliability_invalid_impact_no_escalation", "seconds": 1.5,
                "fault": ("accel_z", "range"),
                "reliability": ("accel_z", 2), "hud": 1,
            },
            {
                "name": "reliability_invalid_visibility_no_escalation", "seconds": 1.5,
                "fault": ("lux", "range"),
                "reliability": ("lux", 2), "hud": 1,
            },
            {
                "name": "reliability_invalid_collision_no_escalation", "seconds": 1.5,
                "fault": ("approach_speed", "range"),
                "reliability": ("approach_speed", 2), "hud": 1,
            },
        ]

        # Exercise every semantically distinct Sensor Faults button through
        # live CARLA -> physical FPGA -> CARLA.  Five combinations above
        # already prove their reliability/risk coupling, so do not repeat
        # them here.  Timeout is global in both the UI and PL, hence one live
        # dropout covers the identical button shown for every sensor.
        already_covered = {
            ("temperature", "jump"),
            ("temperature", "range"),
            ("accel_z", "range"),
            ("lux", "range"),
            ("approach_speed", "range"),
        }
        for sensor, _label in SENSORS:
            checks = ["jump", "stuck", "noise"]
            if sensor in RANGE_SENSORS:
                checks.insert(0, "range")
            if sensor in CONSISTENCY_SENSORS:
                checks.append("consistency")
            for check in checks:
                if (sensor, check) in already_covered:
                    continue
                state = 2 if check == "range" else 1
                seconds = {
                    "range": 0.8,
                    "jump": 0.9,
                    "stuck": 3.5,
                    "noise": 1.2,
                    "consistency": 1.1,
                }[check]
                cases.append({
                    "name": f"sensor_{sensor}_{check}",
                    "seconds": seconds,
                    "fault": (sensor, check),
                    # Stuck alone is DEGRADED; if the dynamics relation also
                    # fails, pack_ch() legitimately promotes it to INVALID.
                    # Both prove that the button reached the intended PL path.
                    ("reliability_min" if check == "stuck" else "reliability"):
                        (sensor, state),
                })
        cases.append({
            "name": "sensor_timeout_global",
            "seconds": 0.35,
            "drop_seconds": 4.0,
            "fault": ("distance", "timeout"),
            "reliability_all": 2,
            "min_samples": 1,
            "any_matching_sample": True,
        })
        return cases

    def _clear(self):
        self.panel.clear_scenarios()
        self.panel.open = True

    def _configure(self, case):
        self._clear()
        self.panel.tab = "risk" if case.get("section") else "sensor"
        section = case.get("section")
        settings = case.get("settings", {})
        if section == "collision":
            self.panel.risk_section = "collision"
            self.panel.collision_request += 1
            self.panel.collision_tier = int(settings[section])
        elif section == "road_surface":
            self.panel.risk_section = "road_surface"
            self.panel.road_surface = ("dry", "wet", "ice", "black_ice")[settings[section]]
        elif section == "road_impact":
            self.panel.risk_section = "road_impact"
            # risk_types.sv deliberately suppresses impact risk below
            # 30 km/h.  Keep the road smooth while the physical CARLA vehicle
            # is brought to a reproducible road-test speed; activate the
            # selected roughness only after that speed is visible to PL.
            self.panel.roughness = 0
            self._pending_impact_roughness = (0, 20, 50, 90)[settings[section]]
            transform = self.world_scenarios.ego.get_transform()
            forward = transform.get_forward_vector()
            target = float(case["min_speed_mps"]) + 1.5
            velocity = self.world_scenarios.ego.get_velocity()
            velocity.x = forward.x * target
            velocity.y = forward.y * target
            velocity.z = forward.z * target
            self.world_scenarios.ego.set_target_velocity(velocity)
        elif section == "visibility_light":
            self.panel.risk_section = "visibility"
            self.panel.visibility_risk = (0, 20, 50, 90)[settings[section]]
        elif section == "visibility_weather":
            self.panel.risk_section = "weather"
            self.panel.weather = ("clear", "fog", "rain", "snow")[settings[section]]
        elif section in {"roll", "yaw", "lateral"}:
            self.panel.risk_section = "posture"
            if section == "roll":
                self.panel.posture[section] = 100
            else:
                self.panel.posture[section] = 25 if settings[section] == 1 else 75

        fault = case.get("fault")
        if fault:
            self.panel.tab = "sensor"
            self.panel.selected_sensor = fault[0]
            self.panel.injector.sensor_faults.add(fault)

    def _advance(self):
        if self.current is not None:
            self._evaluate_current()
        self.index += 1
        if self.index >= len(self.cases):
            self._write_report()
            self._clear()
            self.finished = True
            self.current = None
            return
        self.current = self.cases[self.index]
        self.elapsed = 0.0
        self.observations = []
        self.current_captured = False
        self._precondition_ready = "min_speed_mps" not in self.current
        self._pending_impact_roughness = 0
        self._timeout_released = "drop_seconds" not in self.current
        # Heal diagnostic history and clear a previously latched TD/MRM before
        # the next independent E2E case.  No already-passed case is repeated.
        self._clear()
        self.resetting = True
        self.reset_elapsed = 0.0
        self.force_manual_mode = True
        print(f"[LIVE VERIFY] RESET before {self.current['name']}")

    def update(self, dt, fpga_result, requested_speed_limit, sensor):
        if not self.enabled or self.finished:
            return
        if self.resetting:
            self.reset_elapsed += float(dt)
            if self.reset_elapsed >= 1.2:
                self.resetting = False
                self.force_manual_mode = False
                self._configure(self.current)
                print(f"[LIVE VERIFY] {self.index + 1}/{len(self.cases)} {self.current['name']}")
            return
        if not self._precondition_ready:
            if abs(float(sensor.speed_x)) < self.current["min_speed_mps"]:
                self.elapsed = 0.0
                self.observations = []
                return
            self.panel.roughness = self._pending_impact_roughness
            self.world_scenarios._road_impact_hold_remaining = 30.0
            self._precondition_ready = True
            self.elapsed = 0.0
            self.observations = []
            print(
                f"[LIVE VERIFY] impact precondition ready: "
                f"{abs(float(sensor.speed_x)) * 3.6:.1f} km/h"
            )
            return
        if not self._timeout_released:
            self.elapsed += float(dt)
            if self.elapsed >= self.current["drop_seconds"]:
                self.panel.injector.sensor_faults.discard(("distance", "timeout"))
                self._timeout_released = True
                self.elapsed = 0.0
                self.observations = []
                print("[LIVE VERIFY] timeout dropout complete; observing recovery")
            return
        self.elapsed += float(dt)
        # Ignore scenario transition frames.  Evaluate the final 0.6 seconds.
        observation_start = (
            0.0 if int(self.current.get("min_samples", 5)) == 1
            else max(0.4, self.current["seconds"] - 0.6)
        )
        if fpga_result is not None and self.elapsed >= observation_start:
            self.observations.append((fpga_result, float(requested_speed_limit)))
        if (not self.current_captured and
                self.elapsed >= self.current["seconds"] - 0.15):
            self.capture_due = True
        if self.elapsed >= self.current["seconds"]:
            self._advance()

    def after_draw(self, screen):
        if not self.enabled or not self.capture_due or self.output_dir is None:
            return
        name = self.current["name"] if self.current is not None else "finished"
        pygame.image.save(screen, self.output_dir / f"{self.index + 1:02d}_{name}.png")
        self.capture_due = False
        self.current_captured = True

    @staticmethod
    def _rel_state(word, channel):
        return (int(word) >> (2 * RELIABILITY_INDEX[channel])) & 0x3

    def _matches(self, result, requested_limit):
        case = self.current
        risks = decode_risk_word(result.risk_word)
        failures = []
        if "risk" in case:
            field, expected = case["risk"]
            if risks[field] != expected:
                failures.append(f"{field}={risks[field]} expected={expected}")
        if "risk_min" in case:
            field, minimum = case["risk_min"]
            if risks[field] < minimum:
                failures.append(f"{field}={risks[field]} minimum={minimum}")
        if "reliability" in case:
            channel, expected = case["reliability"]
            actual = self._rel_state(result.reliability_word, channel)
            if actual != expected:
                failures.append(f"{channel}.reliability={actual} expected={expected}")
        if "reliability_min" in case:
            channel, minimum = case["reliability_min"]
            actual = self._rel_state(result.reliability_word, channel)
            if actual < minimum:
                failures.append(
                    f"{channel}.reliability={actual} minimum={minimum}"
                )
        if "reliability_all" in case:
            expected = int(case["reliability_all"])
            for channel in RELIABILITY_INDEX:
                actual = self._rel_state(result.reliability_word, channel)
                if actual != expected:
                    failures.append(
                        f"{channel}.reliability={actual} expected={expected}"
                    )
        control = case.get("control", {})
        if "accelerator_max" in control and result.accelerator > control["accelerator_max"]:
            failures.append(f"accelerator={result.accelerator} max={control['accelerator_max']}")
        if "brake_min" in control and result.brake < control["brake_min"]:
            failures.append(f"brake={result.brake} min={control['brake_min']}")
        if "brake_max" in control and result.brake > control["brake_max"]:
            failures.append(f"brake={result.brake} max={control['brake_max']}")
        for field in ("headlight", "hazard"):
            if field in control and int(getattr(result, field)) != control[field]:
                failures.append(f"{field}={int(getattr(result, field))} expected={control[field]}")
        if "speed_ratio_max" in control:
            maximum = requested_limit * control["speed_ratio_max"] + 0.35
            if result.speed_limit_kmh > maximum:
                failures.append(f"speed_limit={result.speed_limit_kmh:.2f} max={maximum:.2f}")
        if "hud" in case and int(result.hud_warning) != case["hud"]:
            failures.append(f"hud={int(result.hud_warning)} expected={case['hud']}")
        return failures

    def _evaluate_current(self):
        # Normally require five consecutive end-of-stage samples to prove
        # persistence. Timeout is visible only on the first recovery frames,
        # because valid samples immediately heal its saturating counter.
        required = int(self.current.get("min_samples", 5))
        tail = (
            self.observations[:required]
            if self.current.get("take_first_samples")
            else self.observations[-required:]
        )
        sample_failures = [self._matches(result, limit) for result, limit in tail]
        if self.current.get("any_matching_sample"):
            all_failures = [
                self._matches(result, limit)
                for result, limit in self.observations
            ]
            passed = any(not failures for failures in all_failures)
            sample_failures = all_failures
        else:
            passed = len(tail) == required and all(
                not failures for failures in sample_failures
            )
        details = "OK" if passed else (
            f"no {required}-sample FPGA window" if len(tail) < required
            else " | ".join(", ".join(item) for item in sample_failures if item)
        )
        self.results.append({
            "case": self.current["name"], "passed": int(passed),
            "samples": len(self.observations), "details": details,
        })
        print(f"[LIVE VERIFY] {'PASS' if passed else 'FAIL'} {self.current['name']}: {details}")

    def _write_report(self):
        if self.output_dir is None:
            return
        report = self.output_dir / "live_control_results.csv"
        with report.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=("case", "passed", "samples", "details"))
            writer.writeheader()
            writer.writerows(self.results)
        passed = sum(row["passed"] for row in self.results)
        summary = self.output_dir / "summary.txt"
        summary.write_text(
            f"CARLA LIVE CONTROL RESULT: PASS={passed} FAIL={len(self.results) - passed}\n",
            encoding="utf-8",
        )
        print(summary.read_text(encoding="utf-8").strip())
