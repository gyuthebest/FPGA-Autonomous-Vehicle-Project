from enum import Enum, auto


class ScenarioEvent(Enum):
    NONE = auto()

    SCENARIO_START = auto()

    NORMAL_DRIVING = auto()

    FRONT_VEHICLE = auto()

    EMERGENCY_BRAKE = auto()

    VEHICLE_STOP = auto()

    RESTART = auto()

    SCHOOL_ZONE = auto()

    TUNNEL_ENTER = auto()

    TUNNEL_EXIT = auto()

    RAIN_START = auto()

    CURVE = auto()

    MANUAL_REQUEST = auto()

    MANUAL_MODE = auto()

    SCENARIO_END = auto()