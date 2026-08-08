"""
Town04 도로를 자동으로 school/city/mountain/highway로 분류하여
config/zones.json에 저장한다.

기준:
- highway : lane_count >= 3
- mountain: 진행방향 변화(곡률)가 큰 도로 (구불구불함)
- school  : Town04 기본 맵엔 없어 비워둠 (필요시 수동 지정)
- city    : 나머지 전부
"""

import os
import sys
import csv
import json
import math

import carla

HOST = "127.0.0.1"
PORT = 2000

HIGHWAY_LANE_COUNT_THRESHOLD = 3
MOUNTAIN_CURVATURE_THRESHOLD = 8.0  # deg, 인접 waypoint 간 평균 heading 변화


def calculate_curvature(waypoints_by_road):
    """
    같은 road_id의 waypoint들을 순서대로 훑으며
    평균 heading 변화량(deg)을 계산한다.
    """
    result = {}

    for road_id, points in waypoints_by_road.items():

        if len(points) < 3:
            result[road_id] = 0.0
            continue

        points_sorted = sorted(points, key=lambda p: (p[0], p[1]))

        total_diff = 0.0
        count = 0

        for i in range(1, len(points_sorted) - 1):
            x0, y0, yaw0 = points_sorted[i - 1]
            x1, y1, yaw1 = points_sorted[i]

            diff = abs(yaw1 - yaw0)
            if diff > 180:
                diff = 360 - diff

            total_diff += diff
            count += 1

        result[road_id] = (total_diff / count) if count > 0 else 0.0

    return result


def main():

    client = carla.Client(HOST, PORT)
    client.set_timeout(30.0)

    world = client.get_world()
    carla_map = world.get_map()

    waypoints = carla_map.generate_waypoints(2.0)

    lane_count = {}
    waypoints_by_road = {}

    for wp in waypoints:

        road = wp.road_id

        if road not in lane_count:
            lane_count[road] = set()
            waypoints_by_road[road] = []

        lane_count[road].add(wp.lane_id)

        waypoints_by_road[road].append((
            wp.transform.location.x,
            wp.transform.location.y,
            wp.transform.rotation.yaw
        ))

    curvature = calculate_curvature(waypoints_by_road)

    zones = {
        "school": [],
        "city": [],
        "mountain": [],
        "highway": []
    }

    for road_id in sorted(lane_count.keys()):

        n_lanes = len(lane_count[road_id])
        curve = curvature.get(road_id, 0.0)

        if n_lanes >= HIGHWAY_LANE_COUNT_THRESHOLD:
            zones["highway"].append(road_id)
        elif curve >= MOUNTAIN_CURVATURE_THRESHOLD:
            zones["mountain"].append(road_id)
        else:
            zones["city"].append(road_id)

    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    config_dir = os.path.join(project_root, "config")
    os.makedirs(config_dir, exist_ok=True)

    zone_file = os.path.join(config_dir, "zones.json")

    with open(zone_file, "w", encoding="utf-8") as f:
        json.dump(zones, f, indent=4)

    print("=" * 50)
    print("Auto Zone Classification Finished")
    print("=" * 50)
    print(f"Highway  : {len(zones['highway'])} roads  {zones['highway']}")
    print(f"Mountain : {len(zones['mountain'])} roads  {zones['mountain']}")
    print(f"School   : {len(zones['school'])} roads")
    print(f"City     : {len(zones['city'])} roads")
    print()
    print(f"Saved to: {zone_file}")


if __name__ == "__main__":
    main()