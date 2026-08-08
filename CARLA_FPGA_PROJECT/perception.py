"""
==========================================================
CARLA FPGA Autonomous Driving Project

Perception Data
==========================================================
"""


class Perception:

    def __init__(self):

        # -----------------------------
        # Front Object
        # -----------------------------
        self.front_distance = 999.0
        self.front_actor = None
        self.relative_speed = 0.0