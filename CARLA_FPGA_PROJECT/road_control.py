from dataclasses import dataclass
from vehicle_command import VehicleCommand


@dataclass
class RoadControl:

    risk: int = VehicleCommand.RISK_LOW

    throttle: int = 10

    steering_rate: int = 100

    speed_limit: int = 999

    gear_down: bool = False

    brake: int = 0