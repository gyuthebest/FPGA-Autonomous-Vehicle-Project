import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from weather_manager import WeatherManager
import carla
import pygame
import json

HOST = "127.0.0.1"
PORT = 2000

WIDTH = 1400
HEIGHT = 900

zoom = 1.0

offset_x = 0
offset_y = 0

pygame.init()

screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Town04 Viewer")

client = carla.Client(HOST, PORT)
client.set_timeout(20)

world = client.get_world()
weather_manager = WeatherManager(world)
carla_map = world.get_map()

waypoints = carla_map.generate_waypoints(2.0)

points = []
roads = {}

for wp in waypoints:

    loc = wp.transform.location

    point = {

        "x": loc.x,
        "y": loc.y,
        "road": wp.road_id,
        "lane": wp.lane_id

    }

    points.append(point)

    key = (

        wp.road_id,

        wp.lane_id

    )

    if key not in roads:

        roads[key] = []

    roads[key].append(point)

min_x = min(p['x'] for p in points)
max_x = max(p['x'] for p in points)

min_y = min(p['y'] for p in points)
max_y = max(p['y'] for p in points)

def convert(x, y):

    px = (x - min_x) / (max_x - min_x)

    py = (y - min_y) / (max_y - min_y)

    px *= (WIDTH - 40) * zoom
    py *= (HEIGHT - 40) * zoom

    px += offset_x
    py += offset_y

    return int(px + 300), int(py + 20)

def road_color(road):

    zone = get_zone(road)

    if zone == "school":
        return (255,60,60)

    if zone == "highway":
        return (70,130,255)

    if zone == "mountain":
        return (60,220,60)

    return (180,180,180)

def get_zone(road):

    if road in zones["school"]:
        return "school"

    if road in zones["highway"]:
        return "highway"

    if road in zones["mountain"]:
        return "mountain"

    return "city"

def draw_panel():

    panel_width = 260

    pygame.draw.rect(

        screen,

        (45,45,45),

        (0,0,panel_width,HEIGHT)

    )

    pygame.draw.line(

        screen,

        (100,100,100),

        (panel_width,0),

        (panel_width,HEIGHT),

        2

    )

    y = 20

    title = font.render(

        "ZONE EDITOR",

        True,

        (255,255,0)

    )

    screen.blit(title,(15,y))

    y += 40

    for zone in [

        "school",

        "city",

        "highway",

        "mountain"

    ]:

        text = font.render(

            zone.upper(),

            True,

            (255,255,255)

        )

        screen.blit(

            text,

            (15,y)

        )

        y += 25

        for road in sorted(zones[zone]):

            color = road_color(road)

            road_text = font.render(

                f"Road {road}",

                True,

                color

            )

            screen.blit(

                road_text,

                (30,y)

            )

            y += 20

        y += 15

font = pygame.font.SysFont("consolas", 20)

selected = None
hover = None
dragging = False

last_mouse = (0, 0)

zones = {

    "school": [],
    "city": [],
    "mountain": [],
    "highway": []

}

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
zone_file = os.path.join(PROJECT_ROOT, "config", "zones.json")

if os.path.exists(zone_file):

    with open(
        zone_file,
        encoding="utf-8"
    ) as f:

        zones = json.load(f)

running = True

while running:

    screen.fill((30,30,30))

    draw_panel()

    mouse = pygame.mouse.get_pos()

    hover = None

    best = None
    best_dist = float("inf")

    for p in points:

        px, py = convert(
            p["x"],
            p["y"]
        )

        d = (mouse[0] - px) ** 2 + (mouse[1] - py) ** 2

        if d < best_dist:

            best_dist = d
            best = p

    click_radius = 10 * zoom

    if best and best_dist < click_radius * click_radius:

        hover = best

    for event in pygame.event.get():

        if event.type == pygame.QUIT:

            running = False

        if event.type == pygame.MOUSEWHEEL:

            if event.y > 0:

                zoom *= 1.1

            elif event.y < 0:

                zoom /= 1.1

            zoom = max(
                0.2,
                min(
                    zoom,
                    10.0
                )
            )

        if event.type == pygame.MOUSEBUTTONDOWN:

            if event.button == 3:

                dragging = True
                last_mouse = pygame.mouse.get_pos()
                continue

            if hover:

                selected = hover

            else:

                selected = None

        if event.type == pygame.MOUSEBUTTONUP:

            if event.button == 3:

                dragging = False

        if event.type == pygame.MOUSEMOTION:

            if dragging:

                current = pygame.mouse.get_pos()

                dx = current[0] - last_mouse[0]
                dy = current[1] - last_mouse[1]

                offset_x += dx
                offset_y += dy

                last_mouse = current

        if event.type == pygame.KEYDOWN:

            keys = pygame.key.get_pressed()

            if keys[pygame.K_LCTRL] and event.key == pygame.K_s:

                os.makedirs(
                    "config",
                    exist_ok=True
                )
                
                with open(
                    zone_file,
                    "w",
                    encoding="utf-8"
                ) as f:

                    json.dump(
                        zones,
                        f,
                        indent=4
                    )

                print("zones.json 저장 완료")

            elif selected:

                road = selected["road"]

                for key in zones:

                    if road in zones[key]:

                        zones[key].remove(road)

                if event.key == pygame.K_s:

                    zones["school"].append(road)

                elif event.key == pygame.K_c:

                    zones["city"].append(road)

                elif event.key == pygame.K_h:

                    zones["highway"].append(road)

                elif event.key == pygame.K_m:

                    zones["mountain"].append(road)

                elif event.key == pygame.K_DELETE:

                    pass
        
    for p in points:

        px,py = convert(

            p["x"],
            p["y"]

        )

        radius = 1

        if selected and p["road"] == selected["road"]:

            radius = 4

        pygame.draw.circle(

            screen,

            road_color(
                p["road"]
            ),

            (px,py),

            radius

        )

    if hover:

        text = font.render(

            f"Hover : Road {hover['road']}",

            True,

            (255,255,0)

        )

        screen.blit(
            text,
            (320,80)
        )
    
    if selected:

        road = selected["road"]

        zone = get_zone(road)

        text = font.render(

            f"Road : {road}   Lane : {selected['lane']}",

            True,

            (255,255,0)

        )

        screen.blit(text,(320,20))

        text = font.render(

            f"Zone : {zone.upper()}",

            True,

            (255,255,255)

        )

        screen.blit(text,(320,50))

    help1 = font.render(
        "S:School  C:City  H:Highway  M:Mountain",
        True,
        (220,220,220)
    )

    screen.blit(help1,(320,HEIGHT-60))

    help2 = font.render(
        "Delete:Clear   Ctrl+S:Save",
        True,
        (220,220,220)
    )

    zoom_text = font.render(

        f"Zoom : {zoom:.2f}x",

        True,

        (255,255,0)

    )

    screen.blit(
        zoom_text,
        (WIDTH-180,20)
    )

    screen.blit(help2,(320,HEIGHT-30))

    pygame.display.flip()

pygame.quit()