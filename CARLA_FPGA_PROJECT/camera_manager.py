"""
==========================================================
camera_manager.py

CARLA RGB Camera Manager
==========================================================
"""

import numpy as np
import carla
import math


class CameraManager:

    def __init__(self, world, vehicle, image_width=1280, image_height=720):

        self.world = world
        self.vehicle = vehicle

        self.camera = None
        self.image = None
        
        self.last_camera_mode = None
        self.last_camera_yaw = None
        self.last_camera_pitch = None

        blueprint = world.get_blueprint_library().find(
            "sensor.camera.rgb"
        )

        # Match the CARLA RGB sensor to the pygame display.  Drawing a fixed
        # 640x360 sensor image on a fullscreen monitor makes the scene blurry
        # and leaves most of the display unused.
        blueprint.set_attribute("image_size_x", str(max(1, int(image_width))))
        blueprint.set_attribute("image_size_y", str(max(1, int(image_height))))
        blueprint.set_attribute("fov", "90")

        transform = carla.Transform(
            carla.Location(
                x=-6.0,
                z=2.5
            ),
            carla.Rotation(
                pitch=-10.0
            )
        )

        self.camera = world.spawn_actor(
            blueprint,
            transform,
            attach_to=vehicle
        )

        self.camera.listen(
            self._process_image
        )

    def _process_image(self, image):

        array = np.frombuffer(
            image.raw_data,
            dtype=np.uint8
        )

        array = array.reshape(
            (
                image.height,
                image.width,
                4
            )
        )

        self.image = array[:, :, :3].copy()

    def update_transform(self, mode, yaw, pitch):
        if self.camera is None:
            return

        if (self.last_camera_mode == mode and 
            self.last_camera_yaw == yaw and 
            self.last_camera_pitch == pitch):
            return

        self.last_camera_mode = mode
        self.last_camera_yaw = yaw
        self.last_camera_pitch = pitch

        if mode == "first_person":
            # 차량 중심선 1인칭.  y=-0.4 는 좌측 운전석 오프셋이라 화면이 왼쪽으로
            # 치우쳐 보였다.  y=0.0 으로 차량 세로축 중앙에 맞춘다.
            loc = carla.Location(x=0.5, y=0.0, z=1.2)
            rot = carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
        else:
            # 3인칭 궤도(Orbit) 회전
            yaw_rad = math.radians(yaw)
            loc = carla.Location(
                x=-6.0 * math.cos(yaw_rad),
                y=-6.0 * math.sin(yaw_rad),
                z=2.5
            )
            rot = carla.Rotation(pitch=pitch, yaw=yaw, roll=0.0)

        self.camera.set_transform(carla.Transform(loc, rot))

    def destroy(self):

        if self.camera is not None:

            self.camera.stop()
            self.camera.destroy()
