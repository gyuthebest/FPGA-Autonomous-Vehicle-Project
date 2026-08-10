"""
==========================================================
CARLA FPGA Autonomous Driving Project

Map Manager

위치 기반
- Zone
- Speed Limit

==========================================================
"""
import os
import json

class MapManager:

    CITY = "city"
    SCHOOL = "school"
    MOUNTAIN = "mountain"
    HIGHWAY = "highway"

    def __init__(
        self,
        world
    ):

        self.world = world
        self.map = world.get_map()

        base_dir = os.path.dirname(os.path.abspath(__file__))
        map_name = self.map.name.split('/')[-1]
        
        config_path = os.path.join(base_dir, "config", f"zones_{map_name}.json")
        
        # Town04 호환성 유지: zones.json이 있으면 사용
        if not os.path.exists(config_path) and map_name == "Town04":
            config_path = os.path.join(base_dir, "config", "zones.json")

        if os.path.exists(config_path):
            with open(config_path, encoding="utf-8") as f:
                self.zones = json.load(f)
        else:
            print(f"[Warning] {config_path} not found. Using empty zones.")
            self.zones = {"school": [], "city": [], "mountain": [], "highway": []}

    # ======================================================

    def get_zone(
        self,
        location
    ):

        waypoint = self.get_waypoint(
            location
        )

        road = waypoint.road_id

        if road in self.zones["school"]:
            return self.SCHOOL

        if road in self.zones["mountain"]:
            return self.MOUNTAIN

        if road in self.zones["highway"]:
            return self.HIGHWAY

        return self.CITY

    # ======================================================

    def get_speed_limit(
        self,
        location
    ):

        zone = self.get_zone(location)

        if zone == self.SCHOOL:
            return 30

        if zone == self.MOUNTAIN:
            return 70

        if zone == self.HIGHWAY:
            return 100

        return 50

    def get_waypoint(
        self,
        location
    ):

        return self.map.get_waypoint(
            location,
            project_to_road=True
        )