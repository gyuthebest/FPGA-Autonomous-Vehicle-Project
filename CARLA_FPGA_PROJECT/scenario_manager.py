from scenario_state import ScenarioState
from scenario_event import ScenarioEvent
from vehicle_command import VehicleCommand


class ScenarioManager:
    """
    Scenario FSM

    현재는 FPGA 대신 VehicleCommand를 생성한다.
    FPGA가 완성되면 이 모듈만 제거하면 된다.
    """

    ########################################################
    # Constructor
    ########################################################

    def __init__(self):

        self.current_state = ScenarioState.INIT
        self.previous_state = None

        self.current_event = ScenarioEvent.NONE

        self.elapsed_time = 0.0
        self.state_enter_time = 0.0

        self.frame_count = 0

        self.finished = False

        self.command = VehicleCommand()

        self.state_handlers = {

            ScenarioState.INIT: self._update_init,

            ScenarioState.START: self._update_start,

            ScenarioState.CITY_DRIVE: self._update_city_drive,

            ScenarioState.SCHOOL_ZONE: self._update_school,

            ScenarioState.CITY_ROAD: self._update_city_road,

            ScenarioState.FOLLOW_VEHICLE: self._update_follow_vehicle,

            ScenarioState.CURVE: self._update_curve,

            ScenarioState.RAIN: self._update_rain,

            ScenarioState.EMERGENCY_BRAKE: self._update_emergency,

            ScenarioState.STOP: self._update_stop,

            ScenarioState.MANUAL_REQUEST: self._update_manual_request,

            ScenarioState.MANUAL_MODE: self._update_manual_mode,

            ScenarioState.END: self._update_end,
        }

    ########################################################
    # Public
    ########################################################

    def update(self, dt, sensor):

        self.frame_count += 1
        self.elapsed_time += dt

        self.command.reset()

        handler = self.state_handlers[self.current_state]

        handler(sensor)

        return self.command

    ########################################################
    # Property
    ########################################################

    @property
    def state_time(self):

        return self.elapsed_time - self.state_enter_time

    ########################################################
    # State Change
    ########################################################

    def _change_state(self, next_state, event):

        self.previous_state = self.current_state

        self.current_state = next_state

        self.current_event = event

        self.state_enter_time = self.elapsed_time

    ########################################################
    # Default Command
    ########################################################

    def _set_normal_drive(self):

        self.command.throttle = 6
        self.command.brake = 0
        self.command.steering = VehicleCommand.CENTER_STEERING
        self.command.speed_limit = 50

    ########################################################
    # INIT
    ########################################################

    def _update_init(self, sensor):

        if self.state_time > 1.0:

            self._change_state(
                ScenarioState.START,
                ScenarioEvent.SCENARIO_START
            )

    ########################################################
    # START
    ########################################################

    def _update_start(self, sensor):

        self.command.throttle = 3

        if self.state_time > 3.0:

            self._change_state(
                ScenarioState.CITY_DRIVE,
                ScenarioEvent.NORMAL_DRIVING
            )

    ########################################################
    # CITY
    ########################################################

    def _update_city_drive(self, sensor):

        self._set_normal_drive()

    ########################################################
    # FOLLOW
    ########################################################

    def _update_follow_vehicle(self, sensor):

        self.command.throttle = 5
        self.command.speed_limit = 50
        self.command.final_risk = VehicleCommand.RISK_MEDIUM

        if self.state_time > 15:

            self._change_state(
                ScenarioState.RAIN,
                ScenarioEvent.RAIN_START
            )

        ########################################################
    # EMERGENCY
    ########################################################

    def _update_emergency(self, sensor):

        self.command.throttle = 0
        self.command.brake = VehicleCommand.MAX_BRAKE

        self.command.hazard = True

        self.command.emergency_stop = True

        self.command.final_risk = VehicleCommand.RISK_HIGH

        if self.state_time > 5:

            self._change_state(
                ScenarioState.STOP,
                ScenarioEvent.VEHICLE_STOP
            )

    ########################################################
    # STOP
    ########################################################

    def _update_stop(self, sensor):

        self.command.brake = VehicleCommand.MAX_BRAKE

        if self.state_time > 3:

            self._change_state(
                ScenarioState.END,
                ScenarioEvent.SCENARIO_END
            )

    ########################################################
    # SCHOOL ZONE
    ########################################################

    def _update_school(self, sensor):

        self.command.throttle = 3
        self.command.speed_limit = 30

        if self.state_time > 15:

            self._change_state(
                ScenarioState.CITY_ROAD,
                ScenarioEvent.NORMAL_DRIVING
            )

    ########################################################
    # CITY ROAD
    ########################################################

    def _update_city_road(self, sensor):

        self._set_normal_drive()

        if self.state_time > 15:

            self._change_state(
                ScenarioState.FOLLOW_VEHICLE,
                ScenarioEvent.FRONT_VEHICLE
            )

        ########################################################
    # RAIN
    ########################################################

    def _update_rain(self, sensor):

        self.command.throttle = 4
        self.command.speed_limit = 40

        self.command.final_risk = VehicleCommand.RISK_MEDIUM

        if self.state_time > 15:

            self._change_state(
                ScenarioState.CURVE,
                ScenarioEvent.CURVE
            )

    ########################################################
    # CURVE
    ########################################################

    def _update_curve(self, sensor):

        self.command.throttle = 3

        self.command.steering = 18

        self.command.final_risk = VehicleCommand.RISK_MEDIUM

        if self.state_time > 15:
            # FPGA가 없는 현재는
            # 신뢰도 LOW를 가정하여
            # Manual Request를 발생시킨다.
            self._change_state(
                ScenarioState.MANUAL_REQUEST,
                ScenarioEvent.MANUAL_REQUEST
            )

    ########################################################
    # MANUAL REQUEST
    ########################################################

    def _update_manual_request(self, sensor):

        self.command.manual_request = True

        self.command.throttle = 2

        if self.state_time > 5:

            self._change_state(
                ScenarioState.MANUAL_MODE,
                ScenarioEvent.MANUAL_MODE
            )

    ########################################################
    # MANUAL MODE
    ########################################################

    def _update_manual_mode(self, sensor):

        self.command.manual_mode = True

        self.command.autonomous_control = False

        if self.state_time > 20:

            self._change_state(
                ScenarioState.EMERGENCY_BRAKE,
                ScenarioEvent.EMERGENCY_BRAKE
            )

    ########################################################
    # END
    ########################################################

    def _update_end(self, sensor):

        self.finished = True

        self.command.brake = VehicleCommand.MAX_BRAKE



    #신뢰도
    @property
    def confidence(self):

        if self.current_state in (

            ScenarioState.START,
            ScenarioState.CITY_DRIVE,
            ScenarioState.SCHOOL_ZONE,
            ScenarioState.CITY_ROAD,
        ):
            return "HIGH"

        elif self.current_state in (

            ScenarioState.FOLLOW_VEHICLE,
            ScenarioState.CURVE,
            ScenarioState.RAIN,
        ):
            return "MEDIUM"

        elif self.current_state in (

            ScenarioState.MANUAL_REQUEST,
            ScenarioState.MANUAL_MODE,
        ):
            return "LOW"

        return "HIGH"