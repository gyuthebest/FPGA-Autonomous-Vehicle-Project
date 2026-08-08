"""
==========================================================
CARLA FPGA Autonomous Driving Project

Weather Manager

CLEAR / FOG / RAIN / SNOW 4종을 명시적으로 전환하고,
각 날씨에 대응하는 온도/습도까지 함께 관리한다.
(온습도는 CARLA WeatherParameters에 실제 값이 없어
 각 날씨를 대표하는 고정값으로 지정한다)

태양 고도각(sun_altitude_angle)은 이 클래스가 건드리지 않는다.
낮/밤 사이클은 sensor_manager.py가 전담해서, 날씨 전환 시점에
태양 위치가 갑자기 툭 튀는 것을 방지한다.
==========================================================
"""

import carla
from environment import Environment


class WeatherManager:

    def __init__(self, world):

        self.world = world
        self.current = "clear"

        self.weather_code = Environment.CLEAR
        self.temperature = 22.0
        self.humidity = 30.0

    ########################################################

    def current_sun_altitude(self):
        return self.world.get_weather().sun_altitude_angle

    ########################################################

    def set_clear(self):

        if self.current == "clear":
            return

        weather = carla.WeatherParameters(
            cloudiness=0,
            precipitation=0,
            precipitation_deposits=0,
            wetness=0,
            fog_density=0,
            wind_intensity=5,
            sun_altitude_angle=self.current_sun_altitude()
        )

        self.world.set_weather(weather)

        self.current = "clear"
        self.weather_code = Environment.CLEAR
        self.temperature = 22.0
        self.humidity = 30.0

    ########################################################

    def set_rain(self):

        if self.current == "rain":
            return

        weather = carla.WeatherParameters(
            cloudiness=95,
            precipitation=80,
            precipitation_deposits=70,
            wetness=90,
            fog_density=8,
            wind_intensity=20,
            sun_altitude_angle=self.current_sun_altitude()
        )

        self.world.set_weather(weather)

        self.current = "rain"
        self.weather_code = Environment.RAIN
        self.temperature = 15.0
        self.humidity = 85.0

    ########################################################

    def set_fog(self):

        if self.current == "fog":
            return

        weather = carla.WeatherParameters(
            cloudiness=40,
            precipitation=0,
            precipitation_deposits=0,
            wetness=10,
            fog_density=90,
            fog_distance=5,
            fog_falloff=1.0,
            sun_altitude_angle=self.current_sun_altitude()
        )

        self.world.set_weather(weather)

        self.current = "fog"
        self.weather_code = Environment.FOG
        self.temperature = 12.0
        self.humidity = 90.0

    ########################################################

    def set_snow(self):

        if self.current == "snow":
            return

        weather = carla.WeatherParameters(
            cloudiness=90,
            precipitation=60,
            precipitation_deposits=80,
            wetness=30,
            fog_density=20,
            wind_intensity=30,
            sun_altitude_angle=self.current_sun_altitude()
        )

        self.world.set_weather(weather)

        self.current = "snow"
        self.weather_code = Environment.SNOW
        self.temperature = -3.0
        self.humidity = 80.0