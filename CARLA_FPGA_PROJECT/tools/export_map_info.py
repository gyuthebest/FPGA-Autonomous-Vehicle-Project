"""
Town04 Road Information Exporter
"""

import csv
import carla


HOST = "127.0.0.1"
PORT = 2000
MAP = "Town04"


client = carla.Client(HOST, PORT)
client.set_timeout(30.0)

world = client.load_world(MAP)

carla_map = world.get_map()

waypoints = carla_map.generate_waypoints(2.0)

roads = {}

for wp in waypoints:

    road = wp.road_id

    lane = wp.lane_id

    if road not in roads:

        roads[road] = {

            "road_id": road,
            "lane_ids": set(),
            "points": []
        }

    roads[road]["lane_ids"].add(lane)

    roads[road]["points"].append(

        (
            wp.transform.location.x,
            wp.transform.location.y,
            wp.transform.location.z
        )

    )

with open(
    "town04_roads.csv",
    "w",
    newline=""
) as f:

    writer = csv.writer(f)

    writer.writerow([

        "road_id",
        "lane_count",
        "lane_ids",
        "point_count"

    ])

    for road in sorted(roads):

        writer.writerow([

            road,

            len(roads[road]["lane_ids"]),

            sorted(roads[road]["lane_ids"]),

            len(roads[road]["points"])

        ])

print()

print("="*50)
print("Road Export Finished")
print("="*50)
print()

print(len(roads), "roads found.")