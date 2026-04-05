"""Generate a hand-crafted maze arena SDF for the UGV Navigation Challenge.

Layout: 6 cols x 5 rows, each cell 2.0m x 2.0m (total 12m x 10m).
All corridors are 1.85m wide (2.0 - 0.15 wall thickness).
Robot is ~0.3m wide, so 1.85m corridors give plenty of clearance.

ArUco markers are flush on walls at robot camera height (z=0.12).
Directional signs are on posts in corridors at every decision point.

Output: overwrites arena.sdf in the worlds/ directory.
"""

import os

COLS, ROWS = 6, 5
CELL = 2.0
WALL_THICK = 0.15
WALL_HEIGHT = 1.0
CAMERA_Z = 0.12  # Robot camera height

# ---- Hand-crafted maze layout ------------------------------------------
walls_h = [
    [True,  True,  True,  True,  True,  True],   # r=0 bottom boundary
    [False, False, True,  False, True,  False],   # r=1
    [True,  False, False, True,  False, True],    # r=2
    [False, True,  False, False, True,  False],   # r=3
    [True,  False, True,  False, False, True],    # r=4
    [True,  True,  True,  True,  True,  True],    # r=5 top boundary
]

walls_v = [
    [True,  False, True,  False, True,  False, True],   # r=0
    [True,  True,  False, True,  False, False, True],    # r=1
    [True,  False, True,  False, False, True,  True],    # r=2
    [True,  True,  False, False, True,  False, True],    # r=3
    [True,  False, True,  False, True,  False, True],    # r=4
]

# ---- Print ASCII maze ---------------------------------------------------
print("Maze layout (S=start, G=goal, F=false goal, 0-3=ArUco):")
labels = {
    (0, 0): " S ", (0, 5): " G ",
    (3, 4): " F ",
    (1, 2): " 0 ", (0, 4): " 1 ", (4, 1): " 2 ", (2, 3): " 3 ",
}
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
        line += labels.get((r, c), "   ")
    line += "|" if walls_v[r][COLS] else " "
    print(line)
line = ""
for c in range(COLS):
    line += "+---"
line += "+"
print(line)

# ---- Generate wall collisions and visuals --------------------------------
wall_links: list[str] = []
wall_visuals: list[str] = []
wall_id = 0


def add_wall(x, y, length, vertical=False, thick=WALL_THICK):
    global wall_id
    wall_id += 1
    rot = "1.5708" if vertical else "0"
    wall_links.append(
        f'        <collision name="w{wall_id}">\n'
        f'          <pose>{x:.3f} {y:.3f} {WALL_HEIGHT/2} 0 0 {rot}</pose>\n'
        f'          <geometry><box><size>{length:.3f} {thick} {WALL_HEIGHT}</size></box></geometry>\n'
        f'        </collision>'
    )
    wall_visuals.append(
        f'        <visual name="wv{wall_id}">\n'
        f'          <pose>{x:.3f} {y:.3f} {WALL_HEIGHT/2} 0 0 {rot}</pose>\n'
        f'          <geometry><box><size>{length:.3f} {thick} {WALL_HEIGHT}</size></box></geometry>\n'
        f'          <material><ambient>0.35 0.35 0.35 1</ambient>'
        f'<diffuse>0.4 0.4 0.4 1</diffuse></material>\n'
        f'        </visual>'
    )


# Outer boundary
OUTER_THICK = 0.3
arena_w = COLS * CELL
arena_h = ROWS * CELL
add_wall(arena_w/2, -OUTER_THICK/2, arena_w+1.0, thick=OUTER_THICK)
add_wall(arena_w/2, arena_h+OUTER_THICK/2, arena_w+1.0, thick=OUTER_THICK)
add_wall(-OUTER_THICK/2, arena_h/2, arena_h+1.0, vertical=True, thick=OUTER_THICK)
add_wall(arena_w+OUTER_THICK/2, arena_h/2, arena_h+1.0, vertical=True, thick=OUTER_THICK)

# Internal horizontal walls
for r in range(1, ROWS):
    y = r * CELL
    c = 0
    while c < COLS:
        if walls_h[r][c]:
            start_c = c
            while c < COLS and walls_h[r][c]:
                c += 1
            add_wall((start_c*CELL + c*CELL)/2, y, c*CELL - start_c*CELL)
        else:
            c += 1

# Internal vertical walls
for c in range(1, COLS):
    x = c * CELL
    r = 0
    while r < ROWS:
        if walls_v[r][c]:
            start_r = r
            while r < ROWS and walls_v[r][c]:
                r += 1
            add_wall(x, (start_r*CELL + r*CELL)/2, r*CELL - start_r*CELL, vertical=True)
        else:
            r += 1

print(f"\nGenerated {wall_id} wall segments")


# ---- Helper: ArUco marker FLUSH on a wall --------------------------------
def aruco_on_wall(name, wall_x, wall_y, face_dir):
    """Place ArUco marker flush on a wall surface at camera height.
    face_dir: 'north','south','east','west' - which way the marker faces.
    """
    # Offset marker slightly from wall surface so it's visible
    offset = WALL_THICK/2 + 0.005
    if face_dir == "north":
        pose = f"{wall_x} {wall_y + offset} {CAMERA_Z} {-1.5708} 0 0"
    elif face_dir == "south":
        pose = f"{wall_x} {wall_y - offset} {CAMERA_Z} {1.5708} 0 0"
    elif face_dir == "east":
        pose = f"{wall_x + offset} {wall_y} {CAMERA_Z} 0 {1.5708} 0"
    elif face_dir == "west":
        pose = f"{wall_x - offset} {wall_y} {CAMERA_Z} 0 {-1.5708} 0"
    else:
        pose = f"{wall_x} {wall_y} {CAMERA_Z} 0 0 0"
    return (
        f'    <model name="{name}"><static>true</static>\n'
        f'      <pose>{pose}</pose>\n'
        f'      <link name="l"><visual name="v"><geometry><box>'
        f'<size>0.3 0.3 0.005</size></box></geometry>\n'
        f'        <material><ambient>1 1 1 1</ambient>'
        f'<diffuse>1 1 1 1</diffuse></material></visual></link>\n'
        f'    </model>'
    )


# ---- Helper: Sign on a post in corridor ---------------------------------
def sign_on_post(name, x, y, r, g, b):
    """Sign on a thin post, face at camera height."""
    return (
        f'    <model name="{name}"><static>true</static>\n'
        f'      <pose>{x} {y} 0 0 0 0</pose>\n'
        f'      <link name="post">\n'
        f'        <visual name="post_v"><geometry><cylinder>'
        f'<radius>0.015</radius><length>0.2</length></cylinder></geometry>\n'
        f'          <pose>0 0 0.1 0 0 0</pose>\n'
        f'          <material><ambient>0.4 0.4 0.4 1</ambient>'
        f'<diffuse>0.4 0.4 0.4 1</diffuse></material>\n'
        f'        </visual>\n'
        f'        <visual name="face_v"><geometry><box>'
        f'<size>0.2 0.2 0.005</size></box></geometry>\n'
        f'          <pose>0 0 0.22 0 0 0</pose>\n'
        f'          <material><ambient>{r} {g} {b} 1</ambient>'
        f'<diffuse>{r} {g} {b} 1</diffuse></material>\n'
        f'        </visual>\n'
        f'      </link>\n'
        f'    </model>'
    )


# ---- Build SDF -----------------------------------------------------------
# Cell(c,r) center = (c*2+1, r*2+1)
# Wall at bottom of cell(c,r) is at y = r*2
# Wall at left of cell(c,r) is at x = c*2

sdf = f'''<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="ugv_arena">

    <physics name="1ms" type="ignored">
      <max_step_size>0.01</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.7 0.8 0.95 1</background>
      <shadows>true</shadows>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>6 5 15 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>-0.5 0.3 -0.9</direction>
    </light>
    <light type="directional" name="fill">
      <cast_shadows>false</cast_shadows>
      <pose>6 5 12 0 0 0</pose>
      <diffuse>0.4 0.4 0.4 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>0.5 -0.3 -0.8</direction>
    </light>

    <!-- Ground -->
    <model name="ground"><static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material></visual>
      </link>
    </model>

    <!-- START zone (green) cell(0,0) center=(1,1) -->
    <model name="start_zone"><static>true</static><pose>1 1 0.005 0 0 0</pose>
      <link name="l"><visual name="v"><geometry><box><size>1.6 1.6 0.01</size></box></geometry>
        <material><ambient>0.2 0.8 0.2 1</ambient><diffuse>0.2 0.8 0.2 1</diffuse></material></visual></link>
    </model>

    <!-- TRUE GOAL (gold) cell(5,0) center=(11,1) -->
    <model name="true_goal"><static>true</static><pose>11 1 0.005 0 0 0</pose>
      <link name="l"><visual name="v"><geometry><box><size>1.6 1.6 0.01</size></box></geometry>
        <material><ambient>0.9 0.7 0.1 1</ambient><diffuse>0.9 0.7 0.1 1</diffuse></material></visual></link>
    </model>

    <!-- FALSE GOAL cell(4,3) center=(9,7) -->
    <model name="false_goal"><static>true</static><pose>9 7 0.005 0 0 0</pose>
      <link name="l"><visual name="v"><geometry><box><size>1.6 1.6 0.01</size></box></geometry>
        <material><ambient>0.85 0.65 0.1 1</ambient><diffuse>0.85 0.65 0.1 1</diffuse></material></visual></link>
    </model>

    <!-- ARENA WALLS: collision model -->
    <model name="arena_collision"><static>1</static><pose>0 0 0 0 0 0</pose>
      <link name="collision_link"><pose>0 0 0 0 0 0</pose>
{chr(10).join(wall_links)}
      </link>
    </model>

    <!-- ARENA WALLS: visual model -->
    <model name="arena_visual"><static>1</static><pose>0 0 0 0 0 0</pose>
      <link name="visual_link"><pose>0 0 0 0 0 0</pose>
{chr(10).join(wall_visuals)}
      </link>
    </model>

    <!-- ============================================================ -->
    <!-- ArUco markers FLUSH ON WALLS at camera height                 -->
    <!-- ============================================================ -->
    <!-- ArUco 0: on south wall of cell(2,1), faces north into corridor -->
{aruco_on_wall("aruco_0", 5.0, 2.0, "north")}
    <!-- ArUco 1: on south wall of cell(4,0), faces north near goal -->
{aruco_on_wall("aruco_1", 9.0, 0.0, "north")}
    <!-- ArUco 2: on north wall of cell(1,4), faces south from top -->
{aruco_on_wall("aruco_2", 3.0, 10.0, "south")}
    <!-- ArUco 3: on west wall of cell(3,2), faces east into corridor -->
{aruco_on_wall("aruco_3", 6.0, 5.0, "east")}

    <!-- ============================================================ -->
    <!-- Directional signs at EVERY decision point                     -->
    <!-- Guide the robot through: Start -> ArUco0 -> ArUco3 ->        -->
    <!--   ArUco1 -> Goal                                              -->
    <!-- ============================================================ -->

    <!-- Sign 1: FORWARD at start, guide robot north from cell(0,0) -->
{sign_on_post("sign_fwd_1", 1.0, 1.5, 0, 0.8, 0.8)}
    <!-- Sign 2: RIGHT at cell(0,1), guide robot east -->
{sign_on_post("sign_right_1", 1.0, 3.0, 0, 0, 0.8)}
    <!-- Sign 3: FORWARD at cell(1,1), guide robot east toward ArUco0 -->
{sign_on_post("sign_fwd_2", 3.0, 3.0, 0, 0.8, 0.8)}
    <!-- Sign 4: FORWARD at cell(2,1), past ArUco0, continue east -->
{sign_on_post("sign_fwd_3", 5.0, 3.0, 0, 0.8, 0.8)}
    <!-- Sign 5: RIGHT at cell(3,1), guide south toward ArUco3 area -->
{sign_on_post("sign_right_2", 7.0, 3.0, 0, 0, 0.8)}
    <!-- Sign 6: FORWARD at cell(3,0), guide east toward goal -->
{sign_on_post("sign_fwd_4", 7.0, 1.0, 0, 0.8, 0.8)}
    <!-- Sign 7: LEFT (MISLEADING) at cell(0,3), leads to dead end -->
{sign_on_post("sign_left_trap", 1.0, 7.0, 0, 0.8, 0)}
    <!-- Sign 8: STOP near false goal cell(4,3) -->
{sign_on_post("sign_stop", 9.0, 7.0, 0.8, 0, 0)}
    <!-- Sign 9: GOAL at cell(5,0), near true goal -->
{sign_on_post("sign_goal", 11.0, 1.5, 0.9, 0.5, 0)}
    <!-- Sign 10: INPLACE_ROTATION at cell(4,4) for challenge req -->
{sign_on_post("sign_rotate", 9.0, 9.0, 0.8, 0.8, 0)}
    <!-- Sign 11: FORWARD at cell(4,0) guiding toward goal -->
{sign_on_post("sign_fwd_5", 9.0, 1.0, 0, 0.8, 0.8)}
    <!-- Sign 12: RIGHT at cell(2,2) guiding east -->
{sign_on_post("sign_right_3", 5.0, 5.0, 0, 0, 0.8)}

    <!-- Static obstacles -->
    <model name="obs1"><static>true</static><pose>3.0 5.0 0.15 0 0 0.3</pose>
      <link name="l"><collision name="c"><geometry><box><size>0.3 0.3 0.3</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>0.3 0.3 0.3</size></box></geometry>
          <material><ambient>0.5 0.3 0.1 1</ambient><diffuse>0.5 0.3 0.1 1</diffuse></material></visual></link></model>
    <model name="obs2"><static>true</static><pose>7.0 7.0 0.15 0 0 0.7</pose>
      <link name="l"><collision name="c"><geometry><box><size>0.3 0.3 0.3</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>0.3 0.3 0.3</size></box></geometry>
          <material><ambient>0.5 0.3 0.1 1</ambient><diffuse>0.5 0.3 0.1 1</diffuse></material></visual></link></model>
    <model name="obs3"><static>true</static><pose>5.0 1.0 0.15 0 0 0.5</pose>
      <link name="l"><collision name="c"><geometry><box><size>0.3 0.3 0.3</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>0.3 0.3 0.3</size></box></geometry>
          <material><ambient>0.5 0.3 0.1 1</ambient><diffuse>0.5 0.3 0.1 1</diffuse></material></visual></link></model>
    <model name="obs4"><static>true</static><pose>9.0 5.0 0.15 0 0 1.1</pose>
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
print(f"Arena: {COLS}x{ROWS} grid, cell={CELL}m, corridors={CELL-WALL_THICK:.2f}m")
print(f"ArUco markers: flush on walls at z={CAMERA_Z}m")
print(f"Signs: 12 directional signs on posts")
print(f"Robot spawns at (1.0, 1.0)")
