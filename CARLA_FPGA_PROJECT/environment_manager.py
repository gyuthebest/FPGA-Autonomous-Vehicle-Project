"""
==========================================================
CARLA FPGA Autonomous Driving Project

Environment Manager

Sensor

↓

Environment

==========================================================
"""

from environment import Environment
import carla

class EnvironmentManager:

    def __init__(
        self,
        map_manager
    ):

        self.map_manager = map_manager
        self.environment = Environment()

    # ======================================================

    def update(
        self,
        sensor,
        turn_speed_limit=999.0
    ):

        environment = self.environment

        # --------------------------------------------------
        # Weather
        # --------------------------------------------------

        environment.weather = self.calculate_weather(
            sensor
        )

        # --------------------------------------------------
        # Temperature
        # --------------------------------------------------

        environment.temperature = self.calculate_temperature(
            sensor
        )

        # --------------------------------------------------
        # Humidity
        # --------------------------------------------------

        environment.humidity = self.calculate_humidity(
            sensor
        )

        # --------------------------------------------------
        # Visibility
        # --------------------------------------------------

        environment.visibility = self.calculate_visibility(
            sensor
        )

        # --------------------------------------------------
        # Speed Limit
        # --------------------------------------------------





        environment.speed_limit = self.calculate_speed_limit(
            sensor,
            turn_speed_limit
        )

        return environment

    # ======================================================

    def calculate_weather(
        self,
        sensor
    ):

        return sensor.weather

    # ======================================================

    def calculate_temperature(
        self,
        sensor
    ):

        return sensor.temperature

    # ======================================================

    def calculate_humidity(
        self,
        sensor
    ):

        return sensor.humidity

    # ======================================================

    def calculate_visibility(
        self,
        sensor
    ):

        return sensor.visibility

    # ======================================================

    def calculate_speed_limit(
        self,
        sensor,
        turn_speed_limit
    ):
        location = carla.Location(x=sensor.location_x, y=sensor.location_y, z=sensor.location_z)
        map_limit = self.map_manager.get_speed_limit(location)
        return min(map_limit, turn_speed_limit)