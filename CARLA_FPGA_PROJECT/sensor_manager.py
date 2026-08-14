"""
==========================================================
CARLA FPGA Autonomous Driving Project

sensor_manager.py

CARLA 차량 상태를 읽어오는 클래스 (진짜 물리 센서 IMU/Radar 연동)
==========================================================
"""

import math
import time
import carla
import random
import weakref

import utils


class SensorManager:

    def __init__(self, vehicle: carla.Vehicle):

        self.vehicle = vehicle
        self.world = self.vehicle.get_world()

        # -----------------------------
        # Speed (Ground Truth)
        # -----------------------------
        self.speed = 0.0
        self.speed_x = 0.0
        self.speed_y = 0.0
        self.speed_z = 0.0

        # -----------------------------
        # Acceleration (IMU Sensor)
        # -----------------------------
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0

        # -----------------------------
        # Gyro (IMU Sensor)
        # -----------------------------
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0

        # -----------------------------
        # Location & Rotation (Ground Truth)
        # -----------------------------
        self.location_x = 0.0
        self.location_y = 0.0
        self.location_z = 0.0

        self.incline_x = 0.0 # Roll
        self.incline_y = 0.0 # Pitch
        self.incline_z = 0.0 # Yaw
        self.last_imu_time = 0.0
        # Verification metadata only.  These fields let the capture logger
        # prove that the asynchronous sensor callbacks belong to the CARLA
        # frame packed into the current PL sample.
        self.imu_frame = -1
        self.imu_timestamp = -1.0
        self.radar_frame = -1
        self.radar_timestamp = -1.0

        # -----------------------------
        # Forward Vector
        # -----------------------------
        self.forward_x = 0.0
        self.forward_y = 0.0
        self.forward_z = 0.0

        # -----------------------------
        # Time & Environment Cache
        # -----------------------------
        self.start_time = time.time()
        self.frame_count = 0
        self.cached_sun_altitude = 35.0
        self.cached_cloudiness = 0.0
        self.cached_fog_density = 0.0
        self.SUN_UPDATE_INTERVAL_FRAMES = 30

        # -----------------------------
        # Obstacle (Radar Sensor)
        # -----------------------------
        self.distance = 200.0        # Default max range 200m
        self.approach_speed = 0.0    # m/s

        # -----------------------------
        # Simulation System State
        # -----------------------------
        self.rpm = 0
        self.weather = 0
        self.temperature = 20
        self.humidity = 40
        self.visibility = 100
        self.speed_limit = 50
        self.lux = 3000

        # -----------------------------
        # Hardware Sensors (CARLA)
        # -----------------------------
        self.imu_sensor = None
        self.radar_sensor = None
        self._setup_sensors()

    def _setup_sensors(self):
        bp_lib = self.world.get_blueprint_library()

        # 1. IMU Sensor (가속도/각속도용으로 복구)
        imu_bp = bp_lib.find('sensor.other.imu')
        imu_bp.set_attribute('noise_accel_stddev_x', '0.0')
        imu_bp.set_attribute('noise_accel_stddev_y', '0.0')
        imu_bp.set_attribute('noise_accel_stddev_z', '0.0')
        imu_bp.set_attribute('noise_gyro_stddev_x', '0.0')
        imu_bp.set_attribute('noise_gyro_stddev_y', '0.0')
        imu_bp.set_attribute('noise_gyro_stddev_z', '0.0')

        imu_transform = carla.Transform(carla.Location(x=0.0, y=0.0, z=0.0))
        self.imu_sensor = self.world.spawn_actor(imu_bp, imu_transform, attach_to=self.vehicle)
        
        weak_self = weakref.ref(self)
        self.imu_sensor.listen(lambda data: SensorManager._on_imu_event(weak_self, data))

        # 2. Radar Sensor
        radar_bp = bp_lib.find('sensor.other.radar')
        # ACC-style forward corridor.  A 30x30 degree cone repeatedly selected
        # nearby guardrails/road furniture as the closest target and produced
        # false 4 m / 27 m/s collision emergencies on an empty road.
        radar_bp.set_attribute('horizontal_fov', '8')
        radar_bp.set_attribute('vertical_fov', '4')
        radar_bp.set_attribute('range', '200') # 200m range
        
        radar_transform = carla.Transform(carla.Location(x=2.0, z=1.0)) # Front bumper
        self.radar_sensor = self.world.spawn_actor(radar_bp, radar_transform, attach_to=self.vehicle)
        
        self.radar_sensor.listen(lambda data: SensorManager._on_radar_event(weak_self, data))

    @staticmethod
    def _on_imu_event(weak_self, data):
        self = weak_self()
        if not self:
            return
        
        # Acceleration from IMU includes Gravity (+9.81 on Z when parked)
        self.accel_x = data.accelerometer.x
        self.accel_y = data.accelerometer.y
        self.accel_z = data.accelerometer.z

        # Gyroscope directly from IMU (rad/s)
        self.gyro_x = data.gyroscope.x
        self.gyro_y = data.gyroscope.y
        self.gyro_z = data.gyroscope.z
        self.imu_frame = int(data.frame)
        self.imu_timestamp = float(data.timestamp)

    @staticmethod
    def _on_radar_event(weak_self, data):
        self = weak_self()
        if not self:
            return
        
        min_dist = 200.0
        approach_vel = 0.0
        
        # Find the closest detection inside the ego-lane corridor.  CARLA
        # radar reports reflection points, not classified vehicle objects, so
        # angular FOV alone is insufficient on curves and beside guardrails.
        for detection in data:
            lateral_offset = abs(detection.depth * math.sin(detection.azimuth))
            if (
                lateral_offset <= 0.9
                and math.radians(-1.0) <= detection.altitude <= math.radians(3.0)
                and detection.depth < min_dist
            ):
                min_dist = detection.depth
                # CARLA computes dot(target_velocity - ego_velocity, target_direction),
                # so a closing target in front of the ego vehicle is negative.
                # The PL convention is positive closing speed; invert at the interface.
                approach_vel = max(0.0, -detection.velocity)

        self.distance = min_dist
        self.approach_speed = approach_vel
        self.radar_frame = int(data.frame)
        self.radar_timestamp = float(data.timestamp)

    def update(self):


        # =============================
        # Velocity & Speed (Ground Truth)
        # =============================
        velocity = self.vehicle.get_velocity()
        
        # To align velocity with the vehicle's local frame (X=forward), we must project it.
        # However, for pure speed limit checking, absolute magnitude is used.
        # To keep it simple and accurate to world frame as requested previously, or local frame.
        # The vehicle's forward vector tells us its orientation.
        transform = self.vehicle.get_transform()
        forward = transform.get_forward_vector()
        right = transform.get_right_vector()
        up = transform.get_up_vector()

        # Project world velocity onto vehicle local axes to get vehicle-centric velocity
        self.speed_x = velocity.x * forward.x + velocity.y * forward.y + velocity.z * forward.z
        self.speed_y = velocity.x * right.x   + velocity.y * right.y   + velocity.z * right.z
        self.speed_z = velocity.x * up.x      + velocity.y * up.y      + velocity.z * up.z

        speed_mps = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        self.speed = utils.clamp(utils.mps_to_kmh(speed_mps), 0, 200)
        
        limit = self.vehicle.get_speed_limit()
        self.speed_limit = limit if limit and limit > 0 else 50

        # =============================
        # Location & Rotation & Accel (Ground Truth)
        # =============================
        transform = self.vehicle.get_transform()
        location = transform.location
        rotation = transform.rotation

        self.location_x = location.x
        self.location_y = location.y
        self.location_z = location.z

        forward = transform.get_forward_vector()
        right = transform.get_right_vector()
        up = transform.get_up_vector()
        
        self.forward_x = forward.x
        self.forward_y = forward.y
        self.forward_z = forward.z

        # 신의 기준(Ground Truth) 완벽한 참값 기울기 추출 (±30도 캡핑 유지)
        self.incline_x = utils.clamp(rotation.roll, -30.0, 30.0)
        self.incline_y = utils.clamp(rotation.pitch, -30.0, 30.0)
        self.incline_z = rotation.yaw

        # 가속도와 각속도(Gyro)는 다시 물리 센서(IMU) 이벤트에서 받아옵니다.

        # =============================
        # Day/Night Cycle + Lux (조도)
        # =============================
        
        # 날씨 변화에 즉각 반응하기 위해 매 프레임 파라미터를 읽어온다
        current_weather = self.vehicle.get_world().get_weather()
        self.cached_cloudiness = current_weather.cloudiness
        self.cached_fog_density = current_weather.fog_density

        self.frame_count += 1
        if self.frame_count % self.SUN_UPDATE_INTERVAL_FRAMES == 0:
            current_time = time.time()
            elapsed = current_time - self.start_time
            day_period = 120.0
            sun_altitude = -2.0 + 92.0 * (0.5 + 0.5 * math.sin(elapsed * (2 * math.pi / day_period)))

            current_weather.sun_altitude_angle = sun_altitude
            self.vehicle.get_world().set_weather(current_weather)

            self.cached_sun_altitude = sun_altitude

        sun_factor = utils.clamp((self.cached_sun_altitude + 10.0) / 100.0, 0.0, 1.0)
        
        # 기본 조도 범위: 밤(30) ~ 맑은 한낮(130000)
        base_lux = 30 + sun_factor * 129970
        
        # 구름과 안개에 따른 조도 감소 (구름 최대 80% 감소, 안개 최대 50% 감소)
        cloud_factor = 1.0 - (self.cached_cloudiness / 100.0) * 0.8
        fog_factor = 1.0 - (self.cached_fog_density / 100.0) * 0.5
        
        final_lux = base_lux * cloud_factor * fog_factor
        
        self.lux = utils.clamp(final_lux, 30, 130000)

    def destroy(self):
        if self.imu_sensor is not None:
            self.imu_sensor.destroy()
            self.imu_sensor = None
        if self.radar_sensor is not None:
            self.radar_sensor.destroy()
            self.radar_sensor = None
