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

        # 1. IMU Sensor (제거됨 - Ground Truth 사용)

        # 2. Radar Sensor
        radar_bp = bp_lib.find('sensor.other.radar')
        radar_bp.set_attribute('horizontal_fov', '30')
        radar_bp.set_attribute('vertical_fov', '30')
        radar_bp.set_attribute('range', '200') # 200m range
        
        radar_transform = carla.Transform(carla.Location(x=2.0, z=1.0)) # Front bumper
        self.radar_sensor = self.world.spawn_actor(radar_bp, radar_transform, attach_to=self.vehicle)
        
        self.radar_sensor.listen(lambda data: SensorManager._on_radar_event(weak_self, data))

    # IMU 이벤트 핸들러 제거 ( Ground Truth 방식 사용 )

    @staticmethod
    def _on_radar_event(weak_self, data):
        self = weak_self()
        if not self:
            return
        
        min_dist = 200.0
        approach_vel = 0.0
        
        # Find the closest detection
        for detection in data:
            if detection.depth < min_dist:
                min_dist = detection.depth
                approach_vel = detection.velocity # CARLA radar velocity: positive towards sensor

        self.distance = min_dist
        self.approach_speed = approach_vel

    def update(self):
        # =============================
        # Gyro (Ground Truth - Angular Velocity)
        # =============================
        # 차량 물리 엔진 참값에서 각속도를 추출합니다.
        angular_vel = self.vehicle.get_angular_velocity()
        # CARLA의 get_angular_velocity()는 deg/s를 반환하므로 rad/s로 변환
        self.gyro_x = math.radians(angular_vel.x)
        self.gyro_y = math.radians(angular_vel.y)
        self.gyro_z = math.radians(angular_vel.z)

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

        # IMU 센서를 제거했으므로, 가속도 역시 Ground Truth(물리 엔진 참값)로 받아옴
        accel = self.vehicle.get_acceleration()
        self.accel_x = accel.x
        self.accel_y = accel.y
        # CARLA의 get_acceleration()은 중력가속도(9.81)를 포함하지 않는 순수 이동 가속도임.
        # 방지턱 충격(Road Shock) 감지를 위해 기존 IMU와 동일하게 중력(9.81)이 더해진 상태로 맞춰줌
        self.accel_z = accel.z + 9.81

        # =============================
        # Day/Night Cycle + Lux (조도)
        # =============================
        self.frame_count += 1
        if self.frame_count % self.SUN_UPDATE_INTERVAL_FRAMES == 0:
            current_time = time.time()
            elapsed = current_time - self.start_time
            day_period = 120.0
            sun_altitude = -2.0 + 92.0 * (0.5 + 0.5 * math.sin(elapsed * (2 * math.pi / day_period)))

            weather = self.vehicle.get_world().get_weather()
            weather.sun_altitude_angle = sun_altitude
            self.vehicle.get_world().set_weather(weather)

            self.cached_sun_altitude = sun_altitude
            self.cached_cloudiness = weather.cloudiness
            self.cached_fog_density = weather.fog_density

        sun_factor = utils.clamp((self.cached_sun_altitude + 10.0) / 100.0, 0.0, 1.0)
        base_lux = 150 + sun_factor * 24000
        base_lux -= self.cached_cloudiness * 90
        base_lux -= self.cached_fog_density * 60

        noise = 0.0
        self.lux = utils.clamp(base_lux + noise, 50, 130000)

    def destroy(self):
        if self.imu_sensor is not None:
            self.imu_sensor.destroy()
            self.imu_sensor = None
        if self.radar_sensor is not None:
            self.radar_sensor.destroy()
            self.radar_sensor = None