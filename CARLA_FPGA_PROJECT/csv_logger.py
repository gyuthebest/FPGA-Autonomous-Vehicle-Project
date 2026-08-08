import csv
import os
from datetime import datetime


class CSVLogger:

    def __init__(self):

        os.makedirs("logs", exist_ok=True)

        filename = datetime.now().strftime(
            "logs/drive_%Y%m%d_%H%M%S.csv"
        )

        self.file = open(filename, "w", newline="")

        self.writer = csv.writer(self.file)

        self.writer.writerow([
            "Time",

            # ---------- Vehicle State ----------
            "Speed",
            "RPM",
            "Gear",
            "Steering",

            # ---------- Environment ----------
            "Weather",
            "Temperature",
            "Humidity",
            "Lux",
            "RoadSpeedLimit",

            # ---------- Perception / TTC ----------
            "FrontDistance",
            "DistanceOverRange",
            "ClosingSpeed",
            "TTC",
            "TTCRisk",
            "TTCThrottle",
            "TTCBrake",
            "TTCGearDown",
            "TTCHazard",

            # ---------- Road Logic ----------
            "RoadRisk",
            "RoadSurfaceGrade",
            "RoadShockGrade",
            "RoadThrottle",
            "RoadBrake",
            "RoadSpeedLimitCmd",
            "RoadGearDown",

            # ---------- Vision Logic ----------
            "VisionRisk",
            "VisionLuxGrade",
            "VisionWeatherGrade",
            "VisionThrottle",
            "VisionSpeedLimitCmd",
            "VisionHeadlight",
            "VisionHazard",

            # ---------- Posture Logic ----------
            "PostureRisk",
            "PostureRollGrade",
            "PostureYawGrade",
            "PostureLateralGrade",
            "PostureThrottle",
            "PostureBrake",
            "PostureSteerRateLimit",
            "PostureGearDown",

            # ---------- Fusion (Final Command) ----------
            "FinalRisk",
            "FinalThrottle",
            "FinalBrake",
            "FinalSteering",
            "FinalSteerRateLimit",
            "FinalSpeedLimit",
            "FinalGearDown",
            "Headlight",
            "Hazard",
            "ManualMode",
            "AutonomousControl",
            "EmergencyStop",
        ])

    def log(
        self,
        current_time,

        sensor,
        environment,
        controller,
        perception,

        ttc,
        road,
        vision,
        posture,

        command
    ):

        self.writer.writerow([

            round(current_time, 2),

            # ---------- Vehicle State ----------
            round(sensor.speed, 2),
            controller.current_rpm,
            controller.current_gear + 1,
            command.steering,

            # ---------- Environment ----------
            environment.weather,
            round(environment.temperature, 1),
            round(environment.humidity, 1),
            round(sensor.lux, 1),
            round(environment.speed_limit, 1),

            # ---------- Perception / TTC ----------
            round(perception.front_distance, 2),
            ttc.distance_over_range if ttc else "",
            round(ttc.closing_speed, 2) if ttc else "",
            round(ttc.ttc, 2) if ttc else 999,
            ttc.final_risk if ttc else -1,
            ttc.throttle if ttc else "",
            ttc.brake if ttc else "",
            ttc.gear_down_request if ttc else "",
            ttc.hazard if ttc else "",

            # ---------- Road Logic ----------
            road.final_risk if road else -1,
            road.surface_grade if road else "",
            road.shock_grade if road else "",
            road.throttle if road else "",
            road.brake if road else "",
            road.speed_limit if road else "",
            road.gear_down_request if road else "",

            # ---------- Vision Logic ----------
            vision.final_risk if vision else -1,
            vision.lux_grade if vision else "",
            vision.weather_grade if vision else "",
            vision.throttle if vision else "",
            vision.speed_limit if vision else "",
            vision.headlight if vision else "",
            vision.hazard if vision else "",

            # ---------- Posture Logic ----------
            posture.final_risk if posture else -1,
            posture.roll_grade if posture else "",
            posture.yaw_grade if posture else "",
            posture.lateral_grade if posture else "",
            posture.throttle if posture else "",
            posture.brake if posture else "",
            posture.steering_rate_limit if posture else "",
            posture.gear_down_request if posture else "",

            # ---------- Fusion (Final Command) ----------
            command.final_risk,
            command.throttle,
            command.brake,
            command.steering,
            command.steering_rate_limit,
            command.speed_limit,
            command.gear_down_request,
            command.headlight,
            command.hazard,
            command.manual_mode,
            command.autonomous_control,
            command.emergency_stop,
        ])

    def close(self):
        self.file.close()