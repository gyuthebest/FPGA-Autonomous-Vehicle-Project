"""
==========================================================
CARLA FPGA Autonomous Driving Project

Map Zone Editor Utility
- Pygame을 이용한 대화형 지도 구역 편집기
- 마우스를 클릭하여 해당 도로의 Zone(구역)을 칠하고 저장할 수 있습니다.
==========================================================
"""
import carla
import pygame
import sys
import math
import os
import json

HOST = '127.0.0.1'
PORT = 2000

# 구역 인덱스와 색상 정의
ZONE_TYPES = ["school", "city", "mountain", "highway"]
COLORS = {
    "school": (255, 200, 0),      # 노란색
    "city": (150, 150, 150),      # 회색
    "mountain": (50, 200, 50),    # 녹색
    "highway": (50, 100, 255),    # 파란색
    "none": (50, 50, 50)          # 어두운 회색 (미지정)
}

def load_zones(map_name):
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    config_path = os.path.join(base_dir, "config", f"zones_{map_name}.json")
    if not os.path.exists(config_path) and map_name == "Town04":
        config_path = os.path.join(base_dir, "config", "zones.json")
        
    if os.path.exists(config_path):
        with open(config_path, 'r', encoding='utf-8') as f:
            return json.load(f), config_path
    else:
        return {"school": [], "city": [], "mountain": [], "highway": []}, config_path

def save_zones(zones, config_path):
    with open(config_path, 'w', encoding='utf-8') as f:
        json.dump(zones, f, indent=4)
    print(f"[*] Saved to {config_path}")

def get_road_zone(road_id, zones):
    for ztype in ZONE_TYPES:
        if road_id in zones[ztype]:
            return ztype
    return "none"

def set_road_zone(road_id, zones, target_zone):
    # 기존 구역에서 제거
    for ztype in ZONE_TYPES:
        if road_id in zones[ztype]:
            zones[ztype].remove(road_id)
    # 새 구역에 추가
    if target_zone in ZONE_TYPES:
        if road_id not in zones[target_zone]:
            zones[target_zone].append(road_id)

def main():
    print("Connecting to CARLA...")
    client = carla.Client(HOST, PORT)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()
    map_name = carla_map.name.split('/')[-1]
    
    print(f"Loading map waypoints for {map_name}...")
    waypoints = carla_map.generate_waypoints(5.0)
    
    if not waypoints:
        print("Error: No waypoints found.")
        return

    zones, config_path = load_zones(map_name)

    min_x = min([w.transform.location.x for w in waypoints])
    max_x = max([w.transform.location.x for w in waypoints])
    min_y = min([w.transform.location.y for w in waypoints])
    max_y = max([w.transform.location.y for w in waypoints])

    margin = 50
    width_m = max_x - min_x
    height_m = max_y - min_y
    
    screen_w = 1000
    scale = (screen_w - 2 * margin) / width_m if width_m > 0 else 1.0
    screen_h = int(height_m * scale) + 2 * margin
    
    if screen_h > 900:
        screen_h = 900
        scale = (screen_h - 2 * margin) / height_m

    pygame.init()
    screen = pygame.display.set_mode((screen_w, screen_h))
    pygame.display.set_caption(f"CARLA Map Zone Editor - {map_name}")
    font = pygame.font.SysFont("consolas", 20, bold=True)
    small_font = pygame.font.SysFont("consolas", 14)
    
    pts = []
    for w in waypoints:
        x = margin + (w.transform.location.x - min_x) * scale
        y = margin + (w.transform.location.y - min_y) * scale
        pts.append((int(x), int(y), w.road_id))
        
    current_brush = "city"
    brush_size = 50 # 픽셀 반경 내의 도로를 모두 칠함
    is_painting = False

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_1: current_brush = "school"
                elif event.key == pygame.K_2: current_brush = "city"
                elif event.key == pygame.K_3: current_brush = "mountain"
                elif event.key == pygame.K_4: current_brush = "highway"
                elif event.key == pygame.K_0: current_brush = "none"
                elif event.key == pygame.K_s: save_zones(zones, config_path)
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1: is_painting = True
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1: is_painting = False

        mx, my = pygame.mouse.get_pos()

        if is_painting:
            for (x, y, rid) in pts:
                dist = (x - mx)**2 + (y - my)**2
                if dist < brush_size**2:
                    set_road_zone(rid, zones, current_brush)
                
        screen.fill((20, 20, 20))
        
        # Draw roads
        for (x, y, rid) in pts:
            ztype = get_road_zone(rid, zones)
            color = COLORS.get(ztype, COLORS["none"])
            pygame.draw.circle(screen, color, (x, y), 3)
            
        # Highlight closest
        closest_dist = float('inf')
        closest_rid = None
        closest_pos = None
        for (x, y, rid) in pts:
            dist = (x - mx)**2 + (y - my)**2
            if dist < closest_dist:
                closest_dist = dist
                closest_rid = rid
                closest_pos = (x, y)
                
        if closest_dist < 400 and closest_pos:
            pygame.draw.circle(screen, (255, 255, 255), closest_pos, 5, 2)
            ztype = get_road_zone(closest_rid, zones)
            text_surf = font.render(f"ID: {closest_rid} ({ztype})", True, (255, 255, 255))
            pygame.draw.rect(screen, (0, 0, 0), (mx + 10, my - 30, text_surf.get_width() + 10, 30))
            screen.blit(text_surf, (mx + 15, my - 25))
            
        # Draw brush cursor
        pygame.draw.circle(screen, COLORS[current_brush], (mx, my), brush_size, 1)

        # UI
        ui_lines = [
            f"Map: {map_name} | SAVE: [S] Key",
            "Brushes:",
            f"[1] School   (Yellow) {'<--' if current_brush=='school' else ''}",
            f"[2] City     (Gray)   {'<--' if current_brush=='city' else ''}",
            f"[3] Mountain (Green)  {'<--' if current_brush=='mountain' else ''}",
            f"[4] Highway  (Blue)   {'<--' if current_brush=='highway' else ''}",
            f"[0] Eraser   (Dark)   {'<--' if current_brush=='none' else ''}",
        ]
        
        y_off = 10
        for line in ui_lines:
            surf = small_font.render(line, True, (200, 200, 200))
            screen.blit(surf, (10, y_off))
            y_off += 20

        pygame.display.flip()
        pygame.time.wait(30)
        
    pygame.quit()

if __name__ == '__main__':
    main()
