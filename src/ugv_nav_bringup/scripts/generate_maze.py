"""Generate a guaranteed-solvable maze arena SDF for Gazebo.

Uses DFS to carve passages through a grid, ensuring every cell is
reachable from the start. Then adds challenge elements (ArUco markers,
signs, zones, obstacles, traps).

Output: overwrites arena.sdf in the worlds/ directory.
"""

import random
import os

random.seed(42)  # Reproducible maze

# Grid config
COLS, ROWS = 8, 6
CELL = 1.5  # meters per cell
WALL_THICK = 0.20
WALL_HEIGHT = 1.5

# ---- DFS maze generation ------------------------------------------------

maze_visited = [[False] * COLS for _ in range(ROWS)]
walls_h = [[True] * COLS for _ in range(ROWS + 1)]  # horizontal walls
walls_v = [[True] * (COLS + 1) for _ in range(ROWS)]  # vertical walls


def carve(r: int, c: int) -> None:
    maze_visited[r][c] = True
    dirs = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    random.shuffle(dirs)
    for dr, dc in dirs:
        nr, nc = r + dr, c + dc
        if 0 <= nr < ROWS and 0 <= nc < COLS and not maze_visited[nr][nc]:
            if dr == 1:
                walls_h[r + 1][c] = False
            elif dr == -1:
                walls_h[r][c] = False
            elif dc == 1:
                walls_v[r][c + 1] = False
            elif dc == -1:
                walls_v[r][c] = False
            carve(nr, nc)


carve(0, 0)

# Add extra openings for multiple paths (loops, alternate routes)
for r, c, d in [(2, 3, "h"), (3, 5, "h"), (1, 4, "v"), (4, 2, "v"), (3, 7, "v")]:
    if d == "h" and r <= ROWS:
        walls_h[r][c] = False
    elif d == "v" and c <= COLS:
        walls_v[r][c] = False

# Add walls back for dead-end and loop trap
walls_h[5][0] = True  # Dead end cap in upper-left
walls_v[4][6] = True   # Loop trap wall in upper-right

# ---- Print ASCII maze ---------------------------------------------------

print("Maze layout (S=start, G=goal, F=false goal, 0-3=ArUco):")
for r in range(ROWS - 1, -1, -1):
    line = ""
    for c in range(COLS):
        line += "+"
        line += "---" if walls_h[r + 1][c] else "   "
    line += "+"
    print(line)
    line = ""
    for c in range(COLS):
        line += "|" if walls_v[r][c] else " "
        labels = {
            (0, 0): " S ", (0, 7): " G ", (2, 6): " F ",
            (0, 2): " 0 ", (0, 5): " 1 ", (5, 3): " 2 ", (2, 4): " 3 ",
        }
        line += labels.get((r, c), "   ")
    line += "|" if walls_v[r][COLS] else " "
    print(line)
line = ""
for c in range(COLS):
    line += "+---"
line += "+"
print(line)

# ---- Generate wall SDF links --------------------------------------------

wall_links: list[str] = []
wall_id = 0


def add_wall(
    x: float, y: float, length: float,
    vertical: bool = False, thick: float = WALL_THICK,
) -> None:
    """Each wall is its own static model for reliable collision."""
    global wall_id
    wall_id += 1
    rot = "1.5708" if vertical else "0"
    wall_links.append(
        f'    <model name="w{wall_id}">\n'
        f'      <static>true</static>\n'
        f'      <pose>{x:.3f} {y:.3f} {WALL_HEIGHT / 2} 0 0 {rot}</pose>\n'
        f'      <link name="link">\n'
        f'        <collision name="c"><geometry><box><size>{length:.3f} {thick} {WALL_HEIGHT}</size></box></geometry></collision>\n'
        f'        <visual name="v"><geometry><box><size>{length:.3f} {thick} {WALL_HEIGHT}</size></box></geometry>\n'
        f'          <material><ambient>0.35 0.35 0.35 1</ambient><diffuse>0.4 0.4 0.4 1</diffuse></material>\n'
        f'        </visual>\n'
        f'      </link>\n'
        f'    </model>'
    )


OUTER_THICK = 0.5  # Thick outer walls
OUTER_PAD = 0.5    # Padding beyond grid

# Outer boundary as 4 simple walls (NOT from DFS grid)
# These form a solid perimeter with overlapping corners
arena_w = COLS * CELL
arena_h = ROWS * CELL
# Bottom wall: full width + padding
add_wall(arena_w / 2, -OUTER_THICK / 2, arena_w + 2 * OUTER_PAD, thick=OUTER_THICK)
# Top wall
add_wall(arena_w / 2, arena_h + OUTER_THICK / 2, arena_w + 2 * OUTER_PAD, thick=OUTER_THICK)
# Left wall: full height + padding (extends past top/bottom walls)
add_wall(-OUTER_THICK / 2, arena_h / 2, arena_h + 2 * OUTER_PAD,
         vertical=True, thick=OUTER_THICK)
# Right wall
add_wall(arena_w + OUTER_THICK / 2, arena_h / 2, arena_h + 2 * OUTER_PAD,
         vertical=True, thick=OUTER_THICK)

# Internal walls from DFS grid (skip outer boundary rows/cols)
for r in range(1, ROWS):  # Skip r=0 and r=ROWS (outer boundary)
    y = r * CELL
    c = 0
    while c < COLS:
        if walls_h[r][c]:
            start_c = c
            while c < COLS and walls_h[r][c]:
                c += 1
            x_start = start_c * CELL
            x_end = c * CELL
            add_wall((x_start + x_end) / 2, y, x_end - x_start + 0.15)
        else:
            c += 1

for c in range(1, COLS):  # Skip c=0 and c=COLS (outer boundary)
    x = c * CELL
    r = 0
    while r < ROWS:
        if walls_v[r][c]:
            start_r = r
            while r < ROWS and walls_v[r][c]:
                r += 1
            y_start = start_r * CELL
            y_end = r * CELL
            add_wall(x, (y_start + y_end) / 2, y_end - y_start + 0.15,
                     vertical=True)
        else:
            r += 1

print(f"\nGenerated {wall_id} wall segments")

# ---- Build SDF ----------------------------------------------------------

sdf = f'''<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="ugv_arena">

    <physics name="1ms" type="ignored">
      <max_step_size>0.01</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>
    <plugin filename="libignition-gazebo-physics-system.so" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="libignition-gazebo-user-commands-system.so" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="libignition-gazebo-scene-broadcaster-system.so" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre</render_engine>
    </plugin>
    <plugin filename="ignition-gazebo-contact-system" name="ignition::gazebo::systems::Contact"/>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.7 0.8 0.95 1</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>6 4 15 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>-0.5 0.3 -0.9</direction>
    </light>
    <light type="directional" name="fill">
      <cast_shadows>false</cast_shadows>
      <pose>6 4 12 0 0 0</pose>
      <diffuse>0.4 0.4 0.4 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>0.5 -0.3 -0.8</direction>
    </light>

    <!-- Ground -->
    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- START zone (green) cell(0,0) -->
    <model name="start_zone"><static>true</static>
      <pose>0.75 0.75 0.005 0 0 0</pose>
      <link name="l"><visual name="v"><geometry><box><size>1.2 1.2 0.01</size></box></geometry>
        <material><ambient>0.2 0.8 0.2 1</ambient><diffuse>0.2 0.8 0.2 1</diffuse></material></visual></link>
    </model>

    <!-- TRUE GOAL (gold) cell(7,0) -->
    <model name="true_goal"><static>true</static>
      <pose>10.5 0.75 0.005 0 0 0</pose>
      <link name="l"><visual name="v"><geometry><box><size>1.2 1.2 0.01</size></box></geometry>
        <material><ambient>0.9 0.7 0.1 1</ambient><diffuse>0.9 0.7 0.1 1</diffuse></material></visual></link>
    </model>

    <!-- FALSE GOAL cell(6,2) -->
    <model name="false_goal"><static>true</static>
      <pose>9.75 3.75 0.005 0 0 0</pose>
      <link name="l"><visual name="v"><geometry><box><size>1.2 1.2 0.01</size></box></geometry>
        <material><ambient>0.85 0.65 0.1 1</ambient><diffuse>0.85 0.65 0.1 1</diffuse></material></visual></link>
    </model>

    <!-- ARENA WALLS (each wall is a separate static model) -->
{chr(10).join(wall_links)}

    <!-- ArUco markers (4 white boxes on walls) -->
    <model name="aruco_0"><static>true</static><pose>3.0 0.15 0.7 1.5708 0 0</pose>
      <link name="l"><visual name="v"><geometry><box><size>0.3 0.3 0.01</size></box></geometry>
        <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material></visual></link></model>
    <model name="aruco_1"><static>true</static><pose>7.5 0.15 0.7 1.5708 0 0</pose>
      <link name="l"><visual name="v"><geometry><box><size>0.3 0.3 0.01</size></box></geometry>
        <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material></visual></link></model>
    <model name="aruco_2"><static>true</static><pose>5.25 8.5 0.7 1.5708 0 0</pose>
      <link name="l"><visual name="v"><geometry><box><size>0.3 0.3 0.01</size></box></geometry>
        <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material></visual></link></model>
    <model name="aruco_3"><static>true</static><pose>6.0 3.75 0.7 1.5708 0 0</pose>
      <link name="l"><visual name="v"><geometry><box><size>0.3 0.3 0.01</size></box></geometry>
        <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material></visual></link></model>

    <!-- 6 Directional signs (colored boxes) -->
    <model name="sign_forward"><static>true</static><pose>2.25 1.5 0.7 0 0 0</pose>
      <link name="l"><visual name="v"><geometry><box><size>0.25 0.25 0.01</size></box></geometry>
        <material><ambient>0 0.8 0.8 1</ambient><diffuse>0 0.8 0.8 1</diffuse></material></visual></link></model>
    <model name="sign_right"><static>true</static><pose>3.75 0.75 0.7 0 0 0</pose>
      <link name="l"><visual name="v"><geometry><box><size>0.25 0.25 0.01</size></box></geometry>
        <material><ambient>0 0 0.8 1</ambient><diffuse>0 0 0.8 1</diffuse></material></visual></link></model>
    <model name="sign_left_trap"><static>true</static><pose>0.75 2.25 0.7 0 0 0</pose>
      <link name="l"><visual name="v"><geometry><box><size>0.25 0.25 0.01</size></box></geometry>
        <material><ambient>0 0.8 0 1</ambient><diffuse>0 0.8 0 1</diffuse></material></visual></link></model>
    <model name="sign_stop"><static>true</static><pose>8.25 0.75 0.7 0 0 0</pose>
      <link name="l"><visual name="v"><geometry><box><size>0.25 0.25 0.01</size></box></geometry>
        <material><ambient>0.8 0 0 1</ambient><diffuse>0.8 0 0 1</diffuse></material></visual></link></model>
    <model name="sign_goal"><static>true</static><pose>9.75 0.75 0.7 0 0 0</pose>
      <link name="l"><visual name="v"><geometry><box><size>0.25 0.25 0.01</size></box></geometry>
        <material><ambient>0.9 0.5 0 1</ambient><diffuse>0.9 0.5 0 1</diffuse></material></visual></link></model>
    <model name="sign_rotate"><static>true</static><pose>6.75 6.75 0.7 0 0 0</pose>
      <link name="l"><visual name="v"><geometry><box><size>0.25 0.25 0.01</size></box></geometry>
        <material><ambient>0.8 0.8 0 1</ambient><diffuse>0.8 0.8 0 1</diffuse></material></visual></link></model>

    <!-- Static obstacles (4 brown boxes) -->
    <model name="obs1"><static>true</static><pose>2.25 4.5 0.15 0 0 0.3</pose>
      <link name="l"><collision name="c"><geometry><box><size>0.3 0.3 0.3</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>0.3 0.3 0.3</size></box></geometry>
          <material><ambient>0.5 0.3 0.1 1</ambient><diffuse>0.5 0.3 0.1 1</diffuse></material></visual></link></model>
    <model name="obs2"><static>true</static><pose>5.25 1.5 0.15 0 0 0.7</pose>
      <link name="l"><collision name="c"><geometry><box><size>0.3 0.3 0.3</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>0.3 0.3 0.3</size></box></geometry>
          <material><ambient>0.5 0.3 0.1 1</ambient><diffuse>0.5 0.3 0.1 1</diffuse></material></visual></link></model>
    <model name="obs3"><static>true</static><pose>8.25 5.25 0.15 0 0 0.5</pose>
      <link name="l"><collision name="c"><geometry><box><size>0.3 0.3 0.3</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>0.3 0.3 0.3</size></box></geometry>
          <material><ambient>0.5 0.3 0.1 1</ambient><diffuse>0.5 0.3 0.1 1</diffuse></material></visual></link></model>
    <model name="obs4"><static>true</static><pose>3.75 7.5 0.15 0 0 1.1</pose>
      <link name="l"><collision name="c"><geometry><box><size>0.3 0.3 0.3</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>0.3 0.3 0.3</size></box></geometry>
          <material><ambient>0.5 0.3 0.1 1</ambient><diffuse>0.5 0.3 0.1 1</diffuse></material></visual></link></model>

  </world>
</sdf>'''

out_path = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "worlds", "arena.sdf"
)
with open(out_path, "w") as f:
    f.write(sdf)

print(f"\nSDF written to {out_path}")
print(f"Arena: {COLS}x{ROWS} grid, cell={CELL}m")
print(f"Bounds: x=[0, {COLS * CELL}], y=[0, {ROWS * CELL}]")
print(f"Robot spawns at (0.75, 0.75)")
