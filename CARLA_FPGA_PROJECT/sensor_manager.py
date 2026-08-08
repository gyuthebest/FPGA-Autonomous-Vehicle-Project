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

        # 1. IMU Sensor
        imu_bp = bp_lib.find('sensor.other.imu')
        # Add realistic noise (Optional, currently 0 for testing stability, but can be tweaked)
        imu_bp.set_attribute('noise_accel_stddev_x', '0.01')
        imu_bp.set_attribute('noise_accel_stddev_y', '0.01')
        imu_bp.set_attribute('noise_accel_stddev_z', '0.01')
        imu_bp.set_attribute('noise_gyro_stddev_x', '0.001')
        imu_bp.set_attribute('noise_gyro_stddev_y', '0.001')
        imu_bp.set_attribute('noise_gyro_stddev_z', '0.001')

        imu_transform = carla.Transform(carla.Location(x=0.0, y=0.0, z=0.0)) # Center of vehicle
        self.imu_sensor = self.world.spawn_actor(imu_bp, imu_transform, attach_to=self.vehicle)
        
        weak_self = weakref.ref(self)
        self.imu_sensor.listen(lambda data: SensorManager._on_imu_event(weak_self, data))

        # 2. Radar Sensor
        radar_bp = bp_lib.find('sensor.other.radar')
        radar_bp.set_attribute('horizontal_fov', '30')
        radar_bp.set_attribute('vertical_fov', '30')
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

        # ==========================================
        # Sensor Fusion: Complementary Filter
        # ==========================================
        current_time = data.timestamp
        if self.last_imu_time == 0.0:
            self.last_imu_time = current_time
            return
            
        dt = current_time - self.last_imu_time
        if dt <= 0.0:
            dt = 0.01
        self.last_imu_time = current_time

        # 1. 자이로 각속도 적분을 통한 각도 변화량 (rad/s -> deg/s -> deg)
        gyro_pitch_delta = math.degrees(self.gyro_y) * dt
        gyro_roll_delta = math.degrees(self.gyro_x) * dt

        # 2. 가속도계를 이용한 중력 벡터 기반 각도 추출 (Static Pitch/Roll)
        pitch_accel = math.degrees(math.atan2(-self.accel_x, math.sqrt(self.accel_y**2 + self.accel_z**2)))
        roll_accel = math.degrees(math.atan2(self.accel_y, self.accel_z))

        # 3. 상보 필터(Complementary Filter) 적용
        alpha = 0.98
        self.incline_y = alpha * (self.incline_y + gyro_pitch_delta) + (1.0 - alpha) * pitch_accel
        self.incline_x = alpha * (self.incline_x + gyro_roll_delta) + (1.0 - alpha) * roll_accel

        # 4. Yaw (방향각)은 IMU 내장 지자기 센서(Compass) 값 그대로 사용
        self.incline_z = math.degrees(data.compass)

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
        # Location & Rotation (Ground Truth)
        # =============================
        location = transform.location

        self.location_x = location.x
        self.location_y = location.y
        self.location_z = location.z

        self.forward_x = forward.x
        self.forward_y = forward.y
        self.forward_z = forward.z

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

        noise = random.uniform(-100, 100)
        self.lux = utils.clamp(base_lux + noise, 50, 130000)

    def destroy(self):
        if self.imu_sensor is not None:
            self.imu_sensor.destroy()
            self.imu_sensor = None
        if self.radar_sensor is not None:
            self.radar_sensor.destroy()
            self.radar_sensor = None