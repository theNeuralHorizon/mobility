"""Generate a hand-crafted maze arena SDF for the UGV Navigation Challenge.

Layout: 6 cols x 5 rows, each cell 2.0m x 2.0m (total 12m x 10m).
All corridors are 1.85m wide (2.0 - 0.15 wall thickness).
Robot is ~0.3m wide, so 1.85m corridors give plenty of clearance.

Output: overwrites arena.sdf in the worlds/ directory.
"""

import os

COLS, ROWS = 6, 5
CELL = 2.0
WALL_THICK = 0.15
WALL_HEIGHT = 1.0

# ---- Hand-crafted maze layout ------------------------------------------
# True = wall present, False = opening
# Horizontal walls: walls_h[r][c] = wall on the SOUTH side of cell (c, r)
# walls_h[0] = bottom boundary, walls_h[ROWS] = top boundary

walls_h = [
    [True,  True,  True,  True,  True,  True],   # r=0 bottom boundary
    [False, False, True,  False, True,  False],   # r=1
    [True,  False, False, True,  False, True],    # r=2
    [False, True,  False, False, True,  False],   # r=3
    [True,  False, True,  False, False, True],    # r=4
    [True,  True,  True,  True,  True,  True],    # r=5 top boundary
]

# Vertical walls: walls_v[r][c] = wall on the WEST side of cell (c, r)
# walls_v[r][0] = left boundary, walls_v[r][COLS] = right boundary

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
    (1, 0): " 0 ", (0, 1): " 1 ", (2, 3): " 2 ", (2, 4): " 3 ",
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

wall_models: list[str] = []
wall_id = 0


def add_wall(
    x: float, y: float, length: float,
    vertical: bool = False, thick: float = WALL_THICK,
) -> None:
    """Each wall is its own static model with collision + visual in one link."""
    global wall_id
    wall_id += 1
    rot = "1.5708" if vertical else "0"
    wall_models.append(
        f'    <model name="wall_{wall_id}"><static>true</static>\n'
        f'      <pose>{x:.3f} {y:.3f} {WALL_HEIGHT / 2} 0 0 {rot}</pose>\n'
        f'      <link name="link">\n'
        f'        <collision name="c"><geometry><box>'
        f'<size>{length:.3f} {thick} {WALL_HEIGHT}</size>'
        f'</box></geometry></collision>\n'
        f'        <visual name="v"><geometry><box>'
        f'<size>{length:.3f} {thick} {WALL_HEIGHT}</size>'
        f'</box></geometry>\n'
        f'          <material><ambient>0.35 0.35 0.35 1</ambient>'
        f'<diffuse>0.4 0.4 0.4 1</diffuse></material>\n'
        f'        </visual>\n'
        f'      </link>\n'
        f'    </model>'
    )


# Outer boundary (thick, overlapping corners)
OUTER_THICK = 0.3
arena_w = COLS * CELL
arena_h = ROWS * CELL
add_wall(arena_w / 2, -OUTER_THICK / 2, arena_w + 1.0, thick=OUTER_THICK)
add_wall(arena_w / 2, arena_h + OUTER_THICK / 2, arena_w + 1.0, thick=OUTER_THICK)
add_wall(-OUTER_THICK / 2, arena_h / 2, arena_h + 1.0,
         vertical=True, thick=OUTER_THICK)
add_wall(arena_w + OUTER_THICK / 2, arena_h / 2, arena_h + 1.0,
         vertical=True, thick=OUTER_THICK)

# Internal horizontal walls
for r in range(1, ROWS):
    y = r * CELL
    c = 0
    while c < COLS:
        if walls_h[r][c]:
            start_c = c
            while c < COLS and walls_h[r][c]:
                c += 1
            x0 = start_c * CELL
            x1 = c * CELL
            add_wall((x0 + x1) / 2, y, x1 - x0)
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
            y0 = start_r * CELL
            y1 = r * CELL
            add_wall(x, (y0 + y1) / 2, y1 - y0, vertical=True)
        else:
            r += 1

print(f"\nGenerated {wall_id} wall segments")

# ---- Sign model helper ---------------------------------------------------

def sign_model(
    name: str, x: float, y: float,
    r: float, g: float, b: float,
    facing: str = "south",
) -> str:
    """Colored sign panel at robot camera height, facing the approaching robot.

    The sign is a vertical colored panel (0.20 wide x 0.01 thin x 0.15 tall)
    at z=0.08 (robot camera center height). No textures — the HSV color
    detector works on solid color.  A thin dark post holds it up.

    facing: direction the colored face points toward
      - "south": robot approaches from south (+Y to -Y), sign faces -Y
      - "north": robot approaches from north, sign faces +Y
      - "east":  robot approaches from west, sign faces +X
      - "west":  robot approaches from east, sign faces -X
    """
    yaw = {
        "north": 1.5708,
        "south": -1.5708,
        "east":  0.0,
        "west":  3.14159,
    }.get(facing, -1.5708)

    return (
        f'    <model name="{name}"><static>true</static>\n'
        f'      <pose>{x} {y} 0 0 0 0</pose>\n'
        f'      <link name="post">\n'
        f'        <visual name="post"><geometry><cylinder>'
        f'<radius>0.01</radius><length>0.12</length></cylinder></geometry>\n'
        f'          <pose>0 0 0.06 0 0 0</pose>\n'
        f'          <material><ambient>0.2 0.2 0.2 1</ambient>'
        f'<diffuse>0.2 0.2 0.2 1</diffuse></material>\n'
        f'        </visual>\n'
        f'      </link>\n'
        f'      <link name="sign">\n'
        f'        <visual name="face"><geometry><box>'
        f'<size>0.20 0.01 0.15</size></box></geometry>\n'
        f'          <pose>0 0 0.10 0 0 {yaw:.4f}</pose>\n'
        f'          <material><ambient>{r} {g} {b} 1</ambient>'
        f'<diffuse>{r} {g} {b} 1</diffuse></material>\n'
        f'        </visual>\n'
        f'      </link>\n'
        f'    </model>'
    )


def aruco_model(
    name: str, x: float, y: float, z: float,
    marker_id: int, facing: str = "north",
) -> str:
    """ArUco marker standing upright on a wall, visible from the horizontal plane.

    The marker is a thin plane (0.3 x 0.01 x 0.3) — wide along X, tall along Z,
    thin along Y. This makes it stand upright like a painting on a wall.
    We only need to rotate around Z (yaw) to face the right corridor direction.

    facing: which direction the textured face points toward
      - "north": face points toward +Y (robot approaches from south)
      - "south": face points toward -Y (robot approaches from north)
      - "east":  face points toward +X (robot approaches from west)
      - "west":  face points toward -X (robot approaches from east)
    """
    # Yaw rotation only — marker is already upright via geometry
    yaw = {
        "north": 0.0,
        "south": 3.14159,
        "east":  1.5708,
        "west": -1.5708,
    }.get(facing, 0.0)

    return (
        f'    <model name="{name}"><static>true</static>\n'
        f'      <pose>{x} {y} {z} 0 0 {yaw:.4f}</pose>\n'
        f'      <link name="l"><visual name="v"><geometry><box>'
        f'<size>0.3 0.01 0.3</size></box></geometry>\n'
        f'        <material><diffuse>1 1 1 1</diffuse>\n'
        f'          <pbr><metal>\n'
        f'            <albedo_map>../textures/aruco_{marker_id}.png</albedo_map>\n'
        f'          </metal></pbr>\n'
        f'        </material></visual></link>\n'
        f'    </model>'
    )


# ---- Build SDF -----------------------------------------------------------

# Cell centers: cell(c,r) center = (c*CELL + CELL/2, r*CELL + CELL/2)
# = (c*2+1, r*2+1)
#
# ArUco markers ON wall surfaces:
#   aruco_0: On south face of wall at y=4 (wall w7), x=1.0
#            Robot must explore north to cell(0,1) to see it
#   aruco_1: On west face of wall at x=6 (wall w20), y=3.0
#            Robot must reach cell(2,1) area to see it
#   aruco_2: On south face of wall at y=4 (wall w8), x=7.0
#            Wall w8 is at y=4.0, south face = y=4.0 - thick/2 = 3.925
#            Robot approaches from south (cell 3,1), sees marker facing north
#   aruco_3: On west face of wall at x=10 (wall w23), y=5.0
#            Wall w23 is at x=10.0, west face = x=10.0 - thick/2 = 9.925
#            Robot approaches from west (cell 4,2), sees marker facing east

sdf = f'''<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="ugv_arena">

    <physics name="1ms" type="ignored">
      <max_step_size>0.01</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system"
            name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-contact-system"
            name="gz::sim::systems::Contact"/>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.7 0.8 0.95 1</background>
      <shadows>true</shadows>
      <grid>false</grid>
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

    <!-- Ground (trimmed to arena) -->
    <model name="ground">
      <static>true</static>
      <pose>{arena_w / 2} {arena_h / 2} 0 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal>
          <size>{arena_w + 1} {arena_h + 1}</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal>
          <size>{arena_w + 1} {arena_h + 1}</size></plane></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- START zone (green) cell(0,0) center=(1,1) -->
    <model name="start_zone"><static>true</static>
      <pose>1 1 0.005 0 0 0</pose>
      <link name="l"><visual name="v"><geometry><box>
        <size>1.6 1.6 0.01</size></box></geometry>
        <material><ambient>0.2 0.8 0.2 1</ambient>
          <diffuse>0.2 0.8 0.2 1</diffuse></material>
      </visual></link>
    </model>

    <!-- TRUE GOAL (gold) cell(5,0) center=(11,1) -->
    <model name="true_goal"><static>true</static>
      <pose>11 1 0.005 0 0 0</pose>
      <link name="l"><visual name="v"><geometry><box>
        <size>1.6 1.6 0.01</size></box></geometry>
        <material><ambient>0.9 0.7 0.1 1</ambient>
          <diffuse>0.9 0.7 0.1 1</diffuse></material>
      </visual></link>
    </model>

    <!-- FALSE GOAL cell(4,3) center=(9,7) -->
    <model name="false_goal"><static>true</static>
      <pose>9 7 0.005 0 0 0</pose>
      <link name="l"><visual name="v"><geometry><box>
        <size>1.6 1.6 0.01</size></box></geometry>
        <material><ambient>0.85 0.65 0.1 1</ambient>
          <diffuse>0.85 0.65 0.1 1</diffuse></material>
      </visual></link>
    </model>

    <!-- ARENA WALLS: each wall is its own static model with collision+visual -->
{chr(10).join(wall_models)}

    <!-- ============================================================== -->
    <!-- ArUco markers ON wall surfaces, camera height z=0.15           -->
    <!-- ============================================================== -->

    <!-- aruco_0: On south face of horizontal wall at y=4, x=1.0       -->
    <!-- Robot must explore north to cell(0,1) area to see it         -->
{aruco_model("aruco_0", 1.0, 3.925, 0.15, marker_id=0, facing="north")}

    <!-- aruco_1: On west face of vertical wall at x=6, y=3.0         -->
    <!-- Robot must reach cell(2,1) area to see it                     -->
{aruco_model("aruco_1", 5.925, 3.0, 0.15, marker_id=1, facing="east")}

    <!-- aruco_2: On south face of horizontal wall at y=4, near x=7   -->
    <!-- Robot travels north in cell(3,1), sees this on wall ahead     -->
{aruco_model("aruco_2", 7.0, 3.925, 0.15, marker_id=2, facing="north")}

    <!-- aruco_3: On west face of vertical wall at x=10, near y=5     -->
    <!-- Robot travels east in cell(4,2), sees this on wall ahead      -->
{aruco_model("aruco_3", 9.925, 5.0, 0.15, marker_id=3, facing="east")}

    <!-- ============================================================== -->
    <!-- Direction signs along intended path                            -->
    <!-- Path: (0,0)>(1,0)>(1,1)>(2,1)>(2,2)>(3,2)>(4,2)>(4,1)>(5,1)>(5,0) -->
    <!-- ============================================================== -->

    <!-- Direction signs: solid color panels at camera height (z=0.10) -->
    <!-- Colors match HSV detector: FORWARD=cyan, RIGHT=blue, LEFT=green -->
    <!-- STOP=red, GOAL=orange, INPLACE_ROTATION=purple -->

    <!-- 1. FORWARD at start zone — robot approaches from south -->
{sign_model("sign_fwd_start", 1.0, 1.5, 0, 0.8, 0.8, facing="south")}

    <!-- 2. FORWARD east along row 0 — robot approaches from west -->
{sign_model("sign_fwd_east", 1.5, 1.0, 0, 0.8, 0.8, facing="west")}

    <!-- 3. FORWARD north in cell(1,0) — robot approaches from south -->
{sign_model("sign_fwd_10", 3.0, 1.5, 0, 0.8, 0.8, facing="south")}

    <!-- 4. FORWARD north in cell(1,1) — robot approaches from south -->
{sign_model("sign_fwd_11", 3.0, 3.5, 0, 0.8, 0.8, facing="south")}

    <!-- 5. RIGHT at cell(1,1) junction — robot approaches from west -->
{sign_model("sign_right_11", 3.5, 3.0, 0, 0, 0.8, facing="west")}

    <!-- 6. FORWARD north in cell(2,1) — robot approaches from south -->
{sign_model("sign_fwd_21", 5.0, 3.5, 0, 0.8, 0.8, facing="south")}

    <!-- 7. FORWARD east in cell(2,2) — robot approaches from west -->
{sign_model("sign_fwd_22", 5.0, 5.0, 0, 0.8, 0.8, facing="west")}

    <!-- 8. RIGHT at cell(3,2) — robot approaches from west -->
{sign_model("sign_right_32", 7.5, 5.0, 0, 0, 0.8, facing="west")}

    <!-- 9. FORWARD in cell(4,2) — robot approaches from west -->
{sign_model("sign_fwd_42", 9.0, 5.0, 0, 0.8, 0.8, facing="west")}

    <!-- 10. RIGHT at cell(4,1) — robot approaches from north -->
{sign_model("sign_right_41", 9.0, 3.5, 0, 0, 0.8, facing="north")}

    <!-- 11. FORWARD toward goal in cell(5,1) — robot approaches from west -->
{sign_model("sign_fwd_51", 11.0, 3.0, 0, 0.8, 0.8, facing="west")}

    <!-- 12. GOAL sign at cell(5,0) — robot approaches from north -->
{sign_model("sign_goal", 11.0, 1.5, 0.9, 0.5, 0, facing="north")}

    <!-- 13. MISLEADING LEFT at (0,2) junction — robot approaches from south -->
{sign_model("sign_left_misleading", 1.0, 5.0, 0, 0.8, 0, facing="south")}

    <!-- 14. STOP near false goal — robot approaches from west -->
{sign_model("sign_stop", 9.0, 7.0, 0.8, 0, 0, facing="west")}

    <!-- 15. INPLACE_ROTATION in upper corridor — robot approaches from west -->
{sign_model("sign_inplace_rotation", 5.0, 9.0, 0.6, 0, 0.6, facing="west")}

    <!-- Static obstacles -->
    <model name="obs1"><static>true</static>
      <pose>3.0 5.0 0.15 0 0 0.3</pose>
      <link name="l">
        <collision name="c"><geometry><box><size>0.3 0.3 0.3</size>
          </box></geometry></collision>
        <visual name="v"><geometry><box><size>0.3 0.3 0.3</size>
          </box></geometry>
          <material><ambient>0.5 0.3 0.1 1</ambient>
            <diffuse>0.5 0.3 0.1 1</diffuse></material>
        </visual>
      </link>
    </model>
    <model name="obs2"><static>true</static>
      <pose>7.0 7.0 0.15 0 0 0.7</pose>
      <link name="l">
        <collision name="c"><geometry><box><size>0.3 0.3 0.3</size>
          </box></geometry></collision>
        <visual name="v"><geometry><box><size>0.3 0.3 0.3</size>
          </box></geometry>
          <material><ambient>0.5 0.3 0.1 1</ambient>
            <diffuse>0.5 0.3 0.1 1</diffuse></material>
        </visual>
      </link>
    </model>
    <model name="obs3"><static>true</static>
      <pose>5.0 1.0 0.15 0 0 0.5</pose>
      <link name="l">
        <collision name="c"><geometry><box><size>0.3 0.3 0.3</size>
          </box></geometry></collision>
        <visual name="v"><geometry><box><size>0.3 0.3 0.3</size>
          </box></geometry>
          <material><ambient>0.5 0.3 0.1 1</ambient>
            <diffuse>0.5 0.3 0.1 1</diffuse></material>
        </visual>
      </link>
    </model>
    <model name="obs4"><static>true</static>
      <pose>9.0 9.0 0.15 0 0 1.1</pose>
      <link name="l">
        <collision name="c"><geometry><box><size>0.3 0.3 0.3</size>
          </box></geometry></collision>
        <visual name="v"><geometry><box><size>0.3 0.3 0.3</size>
          </box></geometry>
          <material><ambient>0.5 0.3 0.1 1</ambient>
            <diffuse>0.5 0.3 0.1 1</diffuse></material>
        </visual>
      </link>
    </model>

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
print(f"Corridor width: {CELL - WALL_THICK:.2f}m")
print(f"Bounds: x=[0, {arena_w}], y=[0, {arena_h}]")
print(f"Robot spawns at (1.0, 1.0)")
