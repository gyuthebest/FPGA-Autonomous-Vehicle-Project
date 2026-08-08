"""
==========================================================
CARLA FPGA Autonomous Driving Project

Perception Manager

World
Vehicle

↓

Perception
==========================================================
"""

import math

from perception import Perception


class PerceptionManager:

    def __init__(self, world, vehicle):

        self.world = world
        self.vehicle = vehicle

        self.perception = Perception()

        # 전방 탐지 최대 거리 (m)
        self.max_distance = 120.0

        # 전방 경로를 따라갈 때 한 스텝 거리 (m)
        self.path_step = 3.0

        # 경로 샘플점에서 같은 차선으로 인정할 허용 반경 (m)
        self.lane_width_tolerance = 2.2

    # ======================================================

    def build_forward_path(self, ego_location, ego_forward):
        """
        ego 위치에서 도로를 따라 전방으로 waypoint를 이어가며
        (위치, 진행누적거리, 그 지점의 진행방향) 샘플 목록을 만든다.
        분기점에서는 현 진행방향과 가장 정렬된 waypoint를 선택하므로
        도로 segment 경계나 커브에서도 경로가 끊기지 않는다.
        """

        cursor = self.world.get_map().get_waypoint(
            ego_location,
            project_to_road=True
        )

        path = [(cursor.transform.location, 0.0, cursor.transform.get_forward_vector())]

        distance = 0.0
        current_forward = ego_forward

        while distance < self.max_distance:

            candidates = cursor.next(self.path_step)

            if not candidates:
                break

            nxt = max(
                candidates,
                key=lambda c: (
                    c.transform.get_forward_vector().x * current_forward.x
                    + c.transform.get_forward_vector().y * current_forward.y
                )
            )

            align = (
                nxt.transform.get_forward_vector().x * current_forward.x
                + nxt.transform.get_forward_vector().y * current_forward.y
            )

            if align < 0.0:
                break

            distance += self.path_step
            path.append((nxt.transform.location, distance, nxt.transform.get_forward_vector()))

            current_forward = nxt.transform.get_forward_vector()
            cursor = nxt

        return path

    # ======================================================

    def update(self):

        perception = self.perception

        perception.front_distance = 999.0
        perception.front_actor = None

        ego_transform = self.vehicle.get_transform()
        ego_location = ego_transform.location
        ego_forward = ego_transform.get_forward_vector()

        path = self.build_forward_path(ego_location, ego_forward)

        vehicles = self.world.get_actors().filter("vehicle.*")

        nearest_path_distance = None
        nearest_actor = None

        for actor in vehicles:

            if actor.id == self.vehicle.id:
                continue

            location = actor.get_transform().location

            best_point_distance = None
            best_path_distance = None
            best_forward = None
            best_point_location = None

            for point_location, path_distance, point_forward in path:

                dx = location.x - point_location.x
                dy = location.y - point_location.y

                d = math.sqrt(dx * dx + dy * dy)

                if best_point_distance is None or d < best_point_distance:
                    best_point_distance = d
                    best_path_distance = path_distance
                    best_forward = point_forward
                    best_point_location = point_location

            if best_point_distance is None or best_point_distance > self.lane_width_tolerance:
                continue

            # 매칭된 샘플점 기준으로 진행방향 성분만큼 거리를 보정
            dx = location.x - best_point_location.x
            dy = location.y - best_point_location.y

            forward_offset = dx * best_forward.x + dy * best_forward.y

            path_distance_to_actor = best_path_distance + forward_offset

            if path_distance_to_actor < 0.0:
                continue

            if nearest_path_distance is None or path_distance_to_actor < nearest_path_distance:
                nearest_path_distance = path_distance_to_actor
                nearest_actor = actor

        if nearest_actor is not None:

            ego_extent = self.vehicle.bounding_box.extent.x
            actor_extent = nearest_actor.bounding_box.extent.x

            distance = max(0.0, nearest_path_distance - ego_extent - actor_extent)

            if distance <= self.max_distance:

                perception.front_distance = distance
                perception.front_actor = nearest_actor

                ego_vel = self.vehicle.get_velocity()
                actor_vel = nearest_actor.get_velocity()

                ego_speed = math.sqrt(
                    ego_vel.x**2 + ego_vel.y**2 + ego_vel.z**2
                )
                actor_speed = math.sqrt(
                    actor_vel.x**2 + actor_vel.y**2 + actor_vel.z**2
                )

                perception.relative_speed = max(0.0, ego_speed - actor_speed)

        return perception