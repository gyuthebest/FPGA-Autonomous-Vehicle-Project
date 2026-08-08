from enum import Enum, auto


class ScenarioState(Enum):
    """180초 시나리오 FSM 상태"""

    INIT = auto()

    START = auto()

    CITY_DRIVE = auto()

    FOLLOW_VEHICLE = auto()

    EMERGENCY_BRAKE = auto()

    STOP = auto()

    RESTART = auto()

    SCHOOL_ZONE = auto()

    CITY_ROAD = auto()

    RAIN = auto()

    CURVE = auto()

    MANUAL_REQUEST = auto()

    MANUAL_MODE = auto()

    END = auto()