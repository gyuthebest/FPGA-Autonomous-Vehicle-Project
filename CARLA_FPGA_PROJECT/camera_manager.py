"""
==========================================================
camera_manager.py

CARLA RGB Camera Manager
==========================================================
"""

import numpy as np
import carla


class CameraManager:

    def __init__(self, world, vehicle):

        self.world = world
        self.vehicle = vehicle

        self.camera = None
        self.image = None

        blueprint = world.get_blueprint_library().find(
            "sensor.camera.rgb"
        )

        blueprint.set_attribute("image_size_x", "640")
        blueprint.set_attribute("image_size_y", "360")
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

    def destroy(self):

        if self.camera is not None:

            self.camera.stop()
            self.camera.destroy()