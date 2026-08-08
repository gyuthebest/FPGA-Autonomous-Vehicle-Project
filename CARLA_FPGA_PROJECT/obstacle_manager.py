"""
Obstacle Manager

Map Zone에 따라
차량/보행자를 생성한다.
"""

import random
import carla
import math

from map_manager import MapManager
from vehicle_command import VehicleCommand


class ObstacleManager:

    def __init__(
        self,
        world,
        ego_vehicle,
        map_manager,
        traffic_manager=None
    ):

        self.world = world
        self.ego = ego_vehicle
        self.map_manager = map_manager
        self.traffic_manager = traffic_manager

        self.actors = []
        self.actor_state = {}
        self.actor_timer = {}

        self.ttc_timer = 0.0
        self.respawn_timer = 0.0
        self.wait_respawn = False

        # 다음 장애물 생성까지 대기시간
        self.spawn_interval = 20.0

        # 마지막 생성 시각
        self.last_spawn_time = 0.0

    ######################################################

    def destroy(self):

        for actor in self.actors:
            try:
                actor.set_autopilot(False)
                actor.destroy()
            except:
                pass

        self.actors.clear()
        self.actor_state.clear()
        self.actor_timer.clear()

    ########################################################

    def update_actor_behavior(
        self,
        dt
    ):
        # list()로 감싸서 반복문 도중 remove()가 가능하도록 함
        for actor in list(self.actors):
            try:
                if "vehicle" in actor.type_id:
                    self.update_vehicle_behavior(
                        actor,
                        dt
                    )
                elif "walker" in actor.type_id:
                    self.update_pedestrian_behavior(
                        actor,
                        dt
                    )
            except:
                pass

    ########################################################

    def update_vehicle_behavior(
        self,
        actor,
        dt
    ):
        """
        Autopilot(Traffic Manager)이 실제 주행을 담당한다.
        여기서는 일정 시간 후 정리하여 주기적으로
        새 장애물이 나타나도록 수명만 관리한다.
        """
        self.actor_timer[actor.id] += dt

        if self.actor_timer[actor.id] > 20.0:
            try:
                actor.set_autopilot(False)
                actor.destroy()
            except:
                pass

            self.actors.remove(actor)
            self.actor_state.pop(actor.id, None)
            self.actor_timer.pop(actor.id, None)

    ########################################################

    def update_pedestrian_behavior(
        self,
        actor,
        dt
    ):

        self.actor_timer[actor.id] += dt

        state = self.actor_state[actor.id]

        if state == "WAIT":
            if self.actor_timer[actor.id] > 3.0:
                self.actor_state[actor.id] = "CROSS"
                self.actor_timer[actor.id] = 0.0

        elif state == "CROSS":
            location = actor.get_location()
            location.y += 0.05
            actor.set_location(location)

    ######################################################

    def update(
        self,
        dt,
        ttc_result
    ):
        # ------------------------------------
        # Respawn Timer
        # ------------------------------------

        if self.wait_respawn:
            self.respawn_timer += dt

            if self.respawn_timer >= 5.0:
                self.wait_respawn = False
                self.respawn_timer = 0.0
            else:
                return

        # ------------------------------------
        # TTC Monitor
        # ------------------------------------

        if ttc_result is not None:
            if ttc_result.final_risk != VehicleCommand.RISK_LOW:
                self.ttc_timer += dt
            else:
                # STEP 4: 위험도가 낮아질 때 타이머가 한 번에 0이 되지 않고 서서히 감소함
                self.ttc_timer = 0.0

        if self.ttc_timer >= 5.0:
            self.destroy()
            
            self.wait_respawn = True
            self.ttc_timer = 0.0
            return
            
        zone = self.map_manager.get_zone(
            self.ego.get_location()
        )

        # ------------------------------------
        # Spawn Timer
        # ------------------------------------

        self.last_spawn_time += dt

        if len(self.actors) == 0:

            if self.last_spawn_time >= self.spawn_interval:

                self.last_spawn_time = 0.0

                if zone == "school":
                    self.update_school()
                elif zone == "city":
                    self.update_city()
                elif zone == "highway":
                    self.update_highway()
                elif zone == "mountain":
                    self.update_mountain()

        # ------------------------------------
        # Actor Behavior
        # ------------------------------------

        self.update_actor_behavior(dt)

    ######################################################

    def update_school(self):

        if len(self.actors) > 0:
            return

        ego_wp = self.world.get_map().get_waypoint(
            self.ego.get_location(),
            project_to_road=True
        )

        distance = 30
        next_waypoints = ego_wp.next(distance)

        if not next_waypoints:
            return

        ego_yaw = self.ego.get_transform().rotation.yaw

        target_wp = min(
            next_waypoints,
            key=lambda wp: abs(
                ((
                    wp.transform.rotation.yaw
                    - ego_yaw
                    + 180
                ) % 360) - 180
            )
        )

        transform = target_wp.transform
        transform.location.y += 2.5

        self.spawn_pedestrian(transform)

    ######################################################

    def update_city(self):

        if len(self.actors) > 0:
            return

        ego_wp = self.world.get_map().get_waypoint(
            self.ego.get_location(),
            project_to_road=True
        )

        distance = random.uniform(70, 100)
        next_waypoints = ego_wp.next(distance)

        if not next_waypoints:
            return

        ego_yaw = self.ego.get_transform().rotation.yaw

        target_wp = min(
            next_waypoints,
            key=lambda wp: abs(
                ((
                    wp.transform.rotation.yaw
                    - ego_yaw
                    + 180
                ) % 360) - 180
            )
        )

        transform = target_wp.transform

        self.spawn_vehicle(transform)

    ######################################################

    def update_highway(self):

        if len(self.actors) > 0:
            return

        ego_wp = self.world.get_map().get_waypoint(
            self.ego.get_location(),
            project_to_road=True
        )

        distance = random.uniform(120,180)
        next_waypoints = ego_wp.next(distance)

        if not next_waypoints:
            return

        ego_yaw = self.ego.get_transform().rotation.yaw

        target_wp = min(
            next_waypoints,
            key=lambda wp: abs(
                ((
                    wp.transform.rotation.yaw
                    - ego_yaw
                    + 180
                ) % 360) - 180
            )
        )

        transform = target_wp.transform

        self.spawn_vehicle(transform)

    ######################################################

    def update_mountain(self):

        if len(self.actors) > 0:
            return

        ego_wp = self.world.get_map().get_waypoint(
            self.ego.get_location(),
            project_to_road=True
        )

        distance = random.uniform(45,70)
        next_waypoints = ego_wp.next(distance)

        if not next_waypoints:
            return

        ego_yaw = self.ego.get_transform().rotation.yaw

        target_wp = min(
            next_waypoints,
            key=lambda wp: abs(
                ((
                    wp.transform.rotation.yaw
                    - ego_yaw
                    + 180
                ) % 360) - 180
            )
        )

        transform = target_wp.transform

        self.spawn_vehicle(transform)

    ######################################################

    def spawn_vehicle(
        self,
        transform
    ):

        bp = random.choice(
            self.world.get_blueprint_library().filter(
                "vehicle.*"
            )
        )

        transform.location.z += 0.3

        actor = self.world.try_spawn_actor(
            bp,
            transform
        )

        if actor:
            if self.traffic_manager is not None:
                actor.set_autopilot(True, self.traffic_manager.get_port())
                self.traffic_manager.distance_to_leading_vehicle(actor, 4.0)
            else:
                actor.set_autopilot(True)

            self.last_spawn_time = 0.0
            self.actors.append(actor)
            self.actor_state[actor.id] = "AUTO"
            self.actor_timer[actor.id] = 0.0

        return actor

    ######################################################

    def spawn_pedestrian(
        self,
        transform
    ):

        blueprints = self.world.get_blueprint_library().filter(
            "walker.pedestrian.*"
        )

        bp = random.choice(
            blueprints
        )

        actor = self.world.try_spawn_actor(
            bp,
            transform
        )

        if actor:
            self.last_spawn_time = 0.0      # 추가
            self.actors.append(actor)
            self.actor_state[actor.id] = "WAIT"
            self.actor_timer[actor.id] = 0.0

        return actor

    ######################################################

    def get_spawn_points(self):

        return self.world.get_map().get_spawn_points()