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
        self.spawn_interval = 30.0

        # 마지막 생성 시각 (처음 진입 시 즉시 생성되도록 초기화)
        self.last_spawn_time = 30.0
        
        self.destroy_batch = []

    ######################################################

    def destroy(self):
        batch = []
        for actor in self.actors:
            try:
                actor.set_autopilot(False)
            except:
                pass
            batch.append(carla.command.DestroyActor(actor))

        if batch:
            try:
                temp_client = carla.Client("127.0.0.1", 2000)
                temp_client.apply_batch_sync(batch, False)
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
                elif self.actor_state.get(actor.id) in ("BUMP", "BUMP_HIT", "BUMP_DECAL"):
                    self.update_bump_behavior(actor, dt)
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
            except:
                pass
            self.destroy_batch.append(carla.command.DestroyActor(actor))

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

        if self.actor_timer[actor.id] > 20.0:
            try:
                actor.set_autopilot(False)
            except:
                pass
            self.destroy_batch.append(carla.command.DestroyActor(actor))
            self.actors.remove(actor)
            self.actor_state.pop(actor.id, None)
            self.actor_timer.pop(actor.id, None)
            return

        state = self.actor_state[actor.id]

        if state == "WAIT":
            if self.actor_timer[actor.id] > 3.0:
                self.actor_state[actor.id] = "CROSS"
                self.actor_timer[actor.id] = 0.0

        elif state == "CROSS":
            location = actor.get_location()
            location.y += 0.05
            actor.set_location(location)

    ########################################################

    def update_bump_behavior(
        self,
        actor,
        dt
    ):
        self.actor_timer[actor.id] += dt

        if self.actor_timer[actor.id] > 20.0:
            self.destroy_batch.append(carla.command.DestroyActor(actor))

            self.actors.remove(actor)
            self.actor_state.pop(actor.id, None)
            self.actor_timer.pop(actor.id, None)
            return

        # 방지턱 충격(Impulse) 로직: 
        # 자차가 방지턱 콘 근처(3미터 이내)에 접근하면 차를 강제로 위로 띄워버림
        if self.actor_state.get(actor.id) == "BUMP":
            dist = actor.get_location().distance(self.ego.get_location())
            if dist < 3.0:
                mass = self.ego.get_physics_control().mass
                # 차량 질량에 비례하여 위쪽(z축)으로 강한 힘 부여 (초속 3m/s 정도의 점프)
                impulse = carla.Vector3D(0, 0, mass * 3.0)
                self.ego.add_impulse(impulse)
                
                # 한 번 튕겨오르면 이 콘에서는 다시 튕기지 않도록 상태 변경
                self.actor_state[actor.id] = "BUMP_HIT"

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
            
            current_interval = 15.0 if zone == "city" else self.spawn_interval

            if self.last_spawn_time >= current_interval:

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
        
        if hasattr(self, 'destroy_batch') and self.destroy_batch:
            try:
                temp_client = carla.Client("127.0.0.1", 2000)
                temp_client.apply_batch_sync(self.destroy_batch, False)
            except:
                pass
            self.destroy_batch.clear()

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

        # 방지턱 테스트를 위해 생성 거리를 30~50m로 줄임
        distance = random.uniform(30, 50)
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

        if target_wp.is_intersection:
            return

        transform = target_wp.transform

        # 확실한 테스트를 위해 city 구역에서는 100% 확률로 방지턱 생성
        self.spawn_speed_bump(transform)

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

    def spawn_speed_bump(
        self,
        transform
    ):
        """
        static.prop.box01을 이용해 바닥에 시각적인 방지턱을 만듭니다.
        단, 차가 물리적으로 걸리지 않도록 바닥 깊숙이(z=-0.48) 파묻어서 페인트처럼 만듭니다.
        차가 접근하면 update_bump_behavior에서 강제 Impulse를 줍니다.
        (apply_batch_sync를 통해 한 번에 스폰하여 지터를 최소화)
        """
        box_bp = self.world.get_blueprint_library().find("static.prop.box01")
        cone_bp = self.world.get_blueprint_library().find("static.prop.trafficcone01")
        right_vector = transform.get_right_vector()

        batch = []
        states = []
        is_cone = []

        # 차선 폭(약 3.5m)을 덮기 위해 1m짜리 상자 4개를 이어 붙임 (시각 효과)
        for offset in [-1.5, -0.5, 0.5, 1.5]:
            bump_loc = carla.Location(
                x=transform.location.x + right_vector.x * offset,
                y=transform.location.y + right_vector.y * offset,
                z=transform.location.z - 0.48
            )
            bump_transform = carla.Transform(bump_loc, transform.rotation)
            batch.append(carla.command.SpawnActor(box_bp, bump_transform))
            states.append("BUMP_DECAL")
            is_cone.append(False)

        # 시각적 인지를 위한 트래픽 콘(주황색)을 차량과 닿지 않게 차선 양끝(-2.0, 2.0)에만 배치
        for offset in [-2.0, 2.0]:
            cone_loc = carla.Location(
                x=transform.location.x + right_vector.x * offset,
                y=transform.location.y + right_vector.y * offset,
                z=transform.location.z + 0.1
            )
            cone_transform = carla.Transform(cone_loc, transform.rotation)
            batch.append(carla.command.SpawnActor(cone_bp, cone_transform))
            states.append("BUMP")
            is_cone.append(True)

        # 6번의 RPC 통신 렉을 방지하기 위해 1번의 Batch 연산으로 묶어서 서버로 전송
        try:
            temp_client = carla.Client("127.0.0.1", 2000)
            responses = temp_client.apply_batch_sync(batch, False)
            for i, response in enumerate(responses):
                if not response.error:
                    actor_id = response.actor_id
                    actor = self.world.get_actor(actor_id)
                    if actor:
                        if is_cone[i]:
                            actor.set_simulate_physics(True)
                        self.actors.append(actor)
                        self.actor_state[actor_id] = states[i]
                        self.actor_timer[actor_id] = 0.0
        except:
            pass
                
        self.last_spawn_time = 0.0

    ######################################################

    def get_spawn_points(self):

        return self.world.get_map().get_spawn_points()