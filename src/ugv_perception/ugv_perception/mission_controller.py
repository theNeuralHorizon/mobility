"""Mission controller node for UGV Autonomous Navigation Challenge.

Hybrid right-wall-following + visited-cell tracking state machine that
navigates a map-less arena, detects ArUco markers, interprets directional
signs, and reaches the goal.

Uses a PD controller to maintain a fixed distance from the right wall,
with 5-region LiDAR decomposition for decision making.  An overlay grid
tracks visited cells; when the robot revisits a cell 3+ times the
wall-follower is overridden to break out of loops.

States: EXPLORING, SIGN_FOLLOW, MARKER_APPROACH, GOAL_SEEK, RECOVERY,
        MISSION_COMPLETE

Topics:
  Subscribes:
    /r1_mini/lidar        (sensor_msgs/LaserScan)
    /r1_mini/odom         (nav_msgs/Odometry)
    /ugv/aruco/detections (std_msgs/Int32MultiArray)
    /ugv/sign/direction   (std_msgs/String)
  Publishes:
    /cmd_vel              (geometry_msgs/Twist)
    /ugv/mission/state    (std_msgs/String)
"""

from __future__ import annotations

import math
import time
from collections import defaultdict
from dataclasses import dataclass
from enum import Enum, auto
from typing import Final

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray, String


# ---------------------------------------------------------------------------
#  Constants
# ---------------------------------------------------------------------------

MAX_LINEAR: Final[float] = 0.22
MAX_ANGULAR: Final[float] = 0.6
WALL_DIST: Final[float] = 0.4
KP: Final[float] = 1.0
KI: Final[float] = 0.05
KD: Final[float] = 0.5
INTEGRAL_MAX: Final[float] = 1.0

# Stuck detection
STUCK_TIMEOUT: Final[float] = 7.0
STUCK_MOVE_THRESHOLD: Final[float] = 0.1

# Loop detection (position history based)
LOOP_REVISIT_DIST: Final[float] = 0.8
LOOP_TIME_THRESHOLD: Final[float] = 60.0
LOOP_ACTIVATION_DELAY: Final[float] = 90.0

# Visited-cell grid
CELL_SIZE: Final[float] = 0.5
REVISIT_LIMIT: Final[int] = 3

# Markers
REQUIRED_MARKERS: Final[frozenset[int]] = frozenset({0, 1, 2, 3})

# LiDAR
SAFE_RANGE_MAX: Final[float] = 10.0

# Wall-follower thresholds (proven working for ~0.9m corridors)
FRONT_STOP: Final[float] = 0.25
FRONT_SLOW: Final[float] = 0.40
WALL_CLOSE: Final[float] = 0.30
WALL_FAR: Final[float] = 0.80


# ---------------------------------------------------------------------------
#  Enums / Frozen Data
# ---------------------------------------------------------------------------

class State(Enum):
    """Mission state machine states."""

    EXPLORING = auto()
    SIGN_FOLLOW = auto()
    MARKER_APPROACH = auto()
    GOAL_SEEK = auto()
    RECOVERY = auto()
    MISSION_COMPLETE = auto()


@dataclass(frozen=True)
class PositionRecord:
    """Immutable snapshot of robot position at a point in time."""

    x: float
    y: float
    timestamp: float


@dataclass(frozen=True)
class LidarRegions:
    """Five-region decomposition of LiDAR data."""

    right: float = SAFE_RANGE_MAX
    fright: float = SAFE_RANGE_MAX
    front: float = SAFE_RANGE_MAX
    fleft: float = SAFE_RANGE_MAX
    left: float = SAFE_RANGE_MAX


# ---------------------------------------------------------------------------
#  Pure helpers
# ---------------------------------------------------------------------------

def _clamp(value: float, low: float, high: float) -> float:
    """Clamp *value* between *low* and *high* inclusive."""
    return max(low, min(high, value))


def _yaw_from_quaternion(q) -> float:
    """Extract yaw from a ROS quaternion message."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _safe_min(values: list[float], default: float = SAFE_RANGE_MAX) -> float:
    """Return min of *finite* positive values, or *default*."""
    finite = [v for v in values if math.isfinite(v) and v > 0.01]
    return min(min(finite), default) if finite else default


def _pos_to_cell(x: float, y: float) -> tuple[int, int]:
    """Convert world position to a grid cell index."""
    return (int(math.floor(x / CELL_SIZE)), int(math.floor(y / CELL_SIZE)))


def _regions_from_scan(ranges: list[float]) -> LidarRegions:
    """Build five LiDAR regions from a 360-ray scan.

    Index 0 = behind (-pi), 90 = right (-pi/2), 180 = front (0),
    270 = left (+pi/2), 359 ~ behind (+pi).
    """
    n = len(ranges)
    if n == 0:
        return LidarRegions()

    def _sector(lo: int, hi: int) -> float:
        lo_c = max(0, min(n - 1, lo))
        hi_c = max(0, min(n - 1, hi))
        if lo_c > hi_c:
            lo_c, hi_c = hi_c, lo_c
        return _safe_min(list(ranges[lo_c:hi_c + 1]))

    return LidarRegions(
        right=_sector(70, 110),
        fright=_sector(110, 150),
        front=_sector(150, 210),
        fleft=_sector(210, 250),
        left=_sector(250, 290),
    )


def _wall_follow_cmd(
    regions: LidarRegions,
    prev_error: float,
    integral_error: float = 0.0,
    side: str = "right",
) -> tuple[float, float, float, float]:
    """Compute (linear, angular, new_prev_error, new_integral) using PID wall-follower.

    Supports both right-wall and left-wall following via the 'side' parameter.
    Returns a 4-tuple: (linear, angular, new_prev_error, new_integral_error).
    """
    r = regions

    if r.front < FRONT_STOP:
        turn = MAX_ANGULAR if side == "right" else -MAX_ANGULAR
        return 0.0, turn, prev_error, integral_error

    if r.front < FRONT_SLOW:
        turn = MAX_ANGULAR * 0.6 if side == "right" else -MAX_ANGULAR * 0.6
        return 0.08, turn, prev_error, integral_error

    if side == "right":
        # Right-wall following
        if r.fright < WALL_CLOSE:
            return MAX_LINEAR * 0.7, 0.3, prev_error, integral_error
        if r.right > WALL_FAR and r.fright > WALL_FAR:
            return MAX_LINEAR, -0.15, prev_error, integral_error
        if r.right < SAFE_RANGE_MAX:
            error = WALL_DIST - r.right
            d_error = error - prev_error
            new_integral = _clamp(integral_error + error,
                                  -INTEGRAL_MAX, INTEGRAL_MAX)
            angular = _clamp(KP * error + KI * new_integral + KD * d_error,
                             -MAX_ANGULAR, MAX_ANGULAR)
            return MAX_LINEAR, angular, error, new_integral
        return MAX_LINEAR, -0.2, prev_error, integral_error
    else:
        # Left-wall following (mirror of right)
        if r.fleft < WALL_CLOSE:
            return MAX_LINEAR * 0.7, -0.3, prev_error, integral_error
        if r.left > WALL_FAR and r.fleft > WALL_FAR:
            return MAX_LINEAR, 0.15, prev_error, integral_error
        if r.left < SAFE_RANGE_MAX:
            error = WALL_DIST - r.left
            d_error = error - prev_error
            new_integral = _clamp(integral_error + error,
                                  -INTEGRAL_MAX, INTEGRAL_MAX)
            angular = _clamp(-(KP * error + KI * new_integral + KD * d_error),
                             -MAX_ANGULAR, MAX_ANGULAR)
            return MAX_LINEAR, angular, error, new_integral
        return MAX_LINEAR, 0.2, prev_error, integral_error


# ---------------------------------------------------------------------------
#  Node
# ---------------------------------------------------------------------------

class MissionController(Node):
    """Hybrid wall-following + visited-cell mission controller."""

    def __init__(self) -> None:
        super().__init__("mission_controller")

        # -- state machine --
        self._state: State = State.EXPLORING
        self._prev_state: State = State.EXPLORING

        # -- perception data --
        self._regions: LidarRegions = LidarRegions()
        self._visited_markers: set[int] = set()
        self._current_sign: str | None = None

        # -- odometry --
        self._x: float = 0.0
        self._y: float = 0.0
        self._yaw: float = 0.0

        # -- stuck detection --
        self._start_time: float = time.monotonic()
        self._last_move_time: float = time.monotonic()
        self._last_move_x: float = 0.0
        self._last_move_y: float = 0.0

        # -- PD wall-follow state --
        self._prev_wall_error: float = 0.0
        self._wall_integral_error: float = 0.0
        self._wall_follow_side: str = "right"

        # -- visited-cell tracking --
        self._visited_cells: set[tuple[int, int]] = set()
        self._cell_visit_count: dict[tuple[int, int], int] = defaultdict(int)
        self._last_cell: tuple[int, int] = (0, 0)
        self._override_flipped_cell: tuple[int, int] | None = None

        # -- loop detection (position history) --
        self._position_history: list[PositionRecord] = []
        self._last_record_time: float = 0.0

        # -- recovery --
        self._recovery_index: int = 0
        self._recovery_start: float = 0.0
        self._recovery_target_yaw: float = 0.0
        self._consecutive_stucks: int = 0
        self._last_recovery_x: float = 0.0
        self._last_recovery_y: float = 0.0
        self._post_recovery_until: float = 0.0  # escape drive timer

        # -- sign follow --
        self._sign_target_yaw: float = 0.0
        self._sign_phase: str = "turning"
        self._sign_drive_start: float = 0.0

        # -- sign logging & misleading detection --
        self._sign_log: list[dict] = []
        self._pre_sign_x: float = 0.0
        self._pre_sign_y: float = 0.0
        self._pre_sign_yaw: float = 0.0
        self._active_sign_entry: dict | None = None

        # -- ROS 2 interfaces --
        self.create_subscription(
            LaserScan, "/r1_mini/lidar", self._lidar_cb, 10)
        self.create_subscription(
            Odometry, "/r1_mini/odom", self._odom_cb, 10)
        self.create_subscription(
            Int32MultiArray, "/ugv/aruco/detections", self._aruco_cb, 10)
        self.create_subscription(
            String, "/ugv/sign/direction", self._sign_cb, 10)

        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._state_pub = self.create_publisher(
            String, "/ugv/mission/state", 10)

        self.create_timer(0.1, self._control_loop)

        self.get_logger().info("Mission controller started - state: EXPLORING")

    # ------------------------------------------------------------------
    #  Callbacks
    # ------------------------------------------------------------------

    def _lidar_cb(self, msg: LaserScan) -> None:
        self._regions = _regions_from_scan(list(msg.ranges))

    def _odom_cb(self, msg: Odometry) -> None:
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        self._yaw = _yaw_from_quaternion(msg.pose.pose.orientation)

        self._update_move_tracking()
        self._update_position_history()
        self._update_cell_tracking()

    def _aruco_cb(self, msg: Int32MultiArray) -> None:
        for mid in msg.data:
            if mid in REQUIRED_MARKERS and mid not in self._visited_markers:
                self._visited_markers.add(mid)
                self.get_logger().info(
                    f"ArUco {mid} visited! "
                    f"({len(self._visited_markers)}/{len(REQUIRED_MARKERS)})")
                if self._visited_markers >= REQUIRED_MARKERS:
                    self.get_logger().info("ALL 4 MARKERS COLLECTED!")

    def _sign_cb(self, msg: String) -> None:
        if self._state in (State.RECOVERY, State.MISSION_COMPLETE,
                           State.SIGN_FOLLOW):
            return

        direction = msg.data

        if direction == "GOAL":
            if self._visited_markers >= REQUIRED_MARKERS:
                self._transition(State.GOAL_SEEK)
            else:
                self.get_logger().info(
                    "GOAL sign seen but markers incomplete - ignoring")
            return

        if not self._validate_sign_direction(direction):
            return

        # Check if this sign location was previously flagged as misleading
        for entry in self._sign_log:
            if (entry["direction"] == direction
                    and entry["misleading"]
                    and math.hypot(self._x - entry["x"],
                                   self._y - entry["y"]) < 1.5):
                self.get_logger().warn(
                    f"{direction} sign at ({self._x:.1f},{self._y:.1f})"
                    f" previously flagged MISLEADING - skipping")
                return

        # Log sign encounter and store pre-sign position for backtracking
        self._pre_sign_x = self._x
        self._pre_sign_y = self._y
        self._pre_sign_yaw = self._yaw
        self._active_sign_entry = {
            "direction": direction,
            "x": self._x,
            "y": self._y,
            "misleading": False,
        }
        self._sign_log.append(self._active_sign_entry)
        self.get_logger().info(
            f"SIGN logged: {direction} at ({self._x:.1f},{self._y:.1f})"
            f" [total signs: {len(self._sign_log)}]")

        self._current_sign = direction
        self._transition(State.SIGN_FOLLOW)

    # ------------------------------------------------------------------
    #  Odom helper updates (called from _odom_cb)
    # ------------------------------------------------------------------

    def _update_move_tracking(self) -> None:
        """Update stuck-detection move tracker."""
        dx = self._x - self._last_move_x
        dy = self._y - self._last_move_y
        if math.hypot(dx, dy) > STUCK_MOVE_THRESHOLD:
            self._last_move_time = time.monotonic()
            self._last_move_x = self._x
            self._last_move_y = self._y

    def _update_position_history(self) -> None:
        """Record position every 2 s and prune records older than 120 s."""
        now = time.monotonic()
        if now - self._last_record_time > 2.0:
            self._position_history.append(
                PositionRecord(self._x, self._y, now))
            self._last_record_time = now
            cutoff = now - 120.0
            self._position_history = [
                p for p in self._position_history if p.timestamp > cutoff
            ]

    def _update_cell_tracking(self) -> None:
        """Track which grid cells the robot visits and how often."""
        cell = _pos_to_cell(self._x, self._y)
        if cell != self._last_cell:
            self._visited_cells.add(cell)
            self._cell_visit_count[cell] += 1
            self._last_cell = cell

    # ------------------------------------------------------------------
    #  Sign validation
    # ------------------------------------------------------------------

    def _validate_sign_direction(self, direction: str) -> bool:
        """Reject misleading signs using LiDAR cross-check."""
        r = self._regions
        if direction == "LEFT" and (r.left < 0.4 or r.fleft < 0.4):
            self.get_logger().warn("LEFT sign rejected - wall on left")
            return False
        if direction == "RIGHT" and (r.right < 0.4 or r.fright < 0.4):
            self.get_logger().warn("RIGHT sign rejected - wall on right")
            return False
        if direction == "FORWARD" and r.front < 0.4:
            self.get_logger().warn("FORWARD sign rejected - wall ahead")
            return False
        return True

    # ------------------------------------------------------------------
    #  State transition
    # ------------------------------------------------------------------

    def _transition(self, new_state: State) -> None:
        if new_state == self._state:
            return
        self._prev_state = self._state
        self._state = new_state
        self.get_logger().info(
            f"State: {self._prev_state.name} -> {new_state.name}")

        if new_state == State.SIGN_FOLLOW:
            self._setup_sign_follow()
        elif new_state == State.RECOVERY:
            self._setup_recovery()

        state_msg = String()
        state_msg.data = new_state.name
        self._state_pub.publish(state_msg)

    # ------------------------------------------------------------------
    #  Main control loop
    # ------------------------------------------------------------------

    def _control_loop(self) -> None:
        if self._state == State.MISSION_COMPLETE:
            self._publish_vel(0.0, 0.0)
            return

        if self._check_stuck():
            return

        if self._check_loop():
            return

        handlers: dict = {
            State.EXPLORING: self._do_exploring,
            State.SIGN_FOLLOW: self._do_sign_follow,
            State.GOAL_SEEK: self._do_goal_seek,
            State.RECOVERY: self._do_recovery,
        }
        handler = handlers.get(self._state)
        if handler is not None:
            handler()

    def _check_stuck(self) -> bool:
        """Return True (and enter RECOVERY) if robot is stuck."""
        if self._state == State.RECOVERY:
            return False
        stuck_secs = time.monotonic() - self._last_move_time
        if stuck_secs > STUCK_TIMEOUT:
            self.get_logger().warn(
                f"Stuck for {stuck_secs:.1f}s - entering RECOVERY")
            self._transition(State.RECOVERY)
            return True
        return False

    def _check_loop(self) -> bool:
        """Return True (and enter RECOVERY) if a loop is detected."""
        if self._state != State.EXPLORING:
            return False
        elapsed = time.monotonic() - self._start_time
        if elapsed > LOOP_ACTIVATION_DELAY and self._detect_loop():
            self.get_logger().warn("Loop detected - entering RECOVERY")
            self._transition(State.RECOVERY)
            return True
        return False

    # ------------------------------------------------------------------
    #  EXPLORING: Right-wall following + visited-cell override
    # ------------------------------------------------------------------

    def _do_exploring(self) -> None:
        now = time.monotonic()

        # Post-recovery escape: drive toward widest gap for 5s
        if now < self._post_recovery_until:
            self._do_gap_drive()
            return

        cell = _pos_to_cell(self._x, self._y)
        visits = self._cell_visit_count.get(cell, 0)

        # Override wall-follower when cell is revisited too often
        if visits >= REVISIT_LIMIT:
            self._do_exploring_override()
            return

        linear, angular, new_error, new_integral = _wall_follow_cmd(
            self._regions, self._prev_wall_error,
            self._wall_integral_error, self._wall_follow_side)
        self._prev_wall_error = new_error
        self._wall_integral_error = new_integral
        self._publish_vel(linear, angular)

    def _do_gap_drive(self) -> None:
        """Drive toward the widest open direction (gap-seeking).

        Used after recovery to physically escape from stuck corners
        before resuming wall-following.
        """
        r = self._regions
        # Find the most open direction
        directions = [
            (r.left, MAX_ANGULAR),
            (r.fleft, MAX_ANGULAR * 0.5),
            (r.front, 0.0),
            (r.fright, -MAX_ANGULAR * 0.5),
            (r.right, -MAX_ANGULAR),
        ]
        best_range, best_angular = max(directions, key=lambda d: d[0])

        if r.front < FRONT_STOP:
            # Wall ahead — turn toward best direction
            self._publish_vel(0.0, best_angular if best_angular != 0
                              else MAX_ANGULAR)
        elif r.front < FRONT_SLOW:
            # Getting close — slow down and steer
            self._publish_vel(0.1, best_angular * 0.5)
        else:
            # Open ahead — drive forward with slight bias toward best gap
            self._publish_vel(MAX_LINEAR, best_angular * 0.3)

    def _do_exploring_override(self) -> None:
        """Override wall-follower when current cell revisited too often.

        Flips wall-following side ONCE per cell entry, then steers toward
        open space.  Previous version flipped every tick (10 Hz), causing
        oscillation.
        """
        cell = _pos_to_cell(self._x, self._y)

        # Only flip wall-follow side once when first entering this cell
        if self._override_flipped_cell != cell:
            self._override_flipped_cell = cell
            old_side = self._wall_follow_side
            self._wall_follow_side = (
                "left" if old_side == "right" else "right")
            self._wall_integral_error = 0.0  # reset integral on side switch
            self.get_logger().info(
                f"Cell revisit override: switching wall-follow "
                f"{old_side} -> {self._wall_follow_side}")

        r = self._regions

        if r.front < FRONT_STOP:
            turn = MAX_ANGULAR if self._wall_follow_side == "right" else -MAX_ANGULAR
            self._publish_vel(0.0, turn)
            return

        # Steer toward the open side
        if self._wall_follow_side == "left":
            if r.left > WALL_CLOSE and r.fleft > WALL_CLOSE:
                self._publish_vel(MAX_LINEAR * 0.8, 0.3)
            elif r.front > FRONT_SLOW:
                self._publish_vel(MAX_LINEAR, 0.0)
            else:
                self._publish_vel(0.05, MAX_ANGULAR * 0.7)
        else:
            if r.right > WALL_CLOSE and r.fright > WALL_CLOSE:
                self._publish_vel(MAX_LINEAR * 0.8, -0.3)
            elif r.front > FRONT_SLOW:
                self._publish_vel(MAX_LINEAR, 0.0)
            else:
                self._publish_vel(0.05, -MAX_ANGULAR * 0.7)

    # ------------------------------------------------------------------
    #  SIGN_FOLLOW
    # ------------------------------------------------------------------

    def _setup_sign_follow(self) -> None:
        sign = self._current_sign
        if sign == "LEFT":
            self._sign_target_yaw = _normalize_angle(self._yaw + math.pi / 2)
            self._sign_phase = "turning"
        elif sign == "RIGHT":
            self._sign_target_yaw = _normalize_angle(self._yaw - math.pi / 2)
            self._sign_phase = "turning"
        elif sign == "FORWARD":
            self._sign_phase = "driving"
            self._sign_drive_start = time.monotonic()
        elif sign == "STOP":
            self._sign_phase = "stopped"
            self._sign_drive_start = time.monotonic()
        elif sign == "INPLACE_ROTATION":
            self._sign_target_yaw = _normalize_angle(self._yaw + 2 * math.pi)
            self._sign_phase = "spinning"
            self._sign_drive_start = time.monotonic()

    def _do_sign_follow(self) -> None:
        now = time.monotonic()

        if self._sign_phase == "turning":
            self._do_sign_turning(now)
        elif self._sign_phase == "driving":
            self._do_sign_driving(now)
        elif self._sign_phase == "stopped":
            self._do_sign_stopped(now)
        elif self._sign_phase == "spinning":
            self._do_sign_spinning(now)
        elif self._sign_phase == "backtracking":
            self._do_sign_backtrack(now)

    def _do_sign_turning(self, now: float) -> None:
        error = _normalize_angle(self._sign_target_yaw - self._yaw)
        if abs(error) < 0.15:
            self._sign_phase = "driving"
            self._sign_drive_start = now
            self._publish_vel(0.0, 0.0)
        else:
            angular = _clamp(error * 1.5, -MAX_ANGULAR, MAX_ANGULAR)
            self._publish_vel(0.0, angular)

    def _do_sign_driving(self, now: float) -> None:
        if self._regions.front < 0.4:
            # Hit a wall while following sign - classify as MISLEADING
            if self._active_sign_entry is not None:
                self._active_sign_entry["misleading"] = True
                self.get_logger().warn(
                    f"MISLEADING sign detected: "
                    f"{self._active_sign_entry['direction']} at "
                    f"({self._active_sign_entry['x']:.1f},"
                    f"{self._active_sign_entry['y']:.1f}) - BACKTRACKING")
                self._active_sign_entry = None
                # Backtrack: reverse toward pre-sign position
                self._sign_phase = "backtracking"
                self._sign_drive_start = now
            else:
                self._transition(State.EXPLORING)
            return
        if now - self._sign_drive_start > 2.0:
            # Completed sign driving normally - sign was trustworthy
            self.get_logger().info("Sign follow completed successfully")
            self._active_sign_entry = None
            self._transition(State.EXPLORING)
            return
        self._publish_vel(MAX_LINEAR, 0.0)

    def _do_sign_stopped(self, now: float) -> None:
        if now - self._sign_drive_start > 2.0:
            self._transition(State.EXPLORING)
            return
        self._publish_vel(0.0, 0.0)

    def _do_sign_spinning(self, now: float) -> None:
        if now - self._sign_drive_start > 5.0:
            self._transition(State.EXPLORING)
            return
        self._publish_vel(0.0, MAX_ANGULAR)

    def _do_sign_backtrack(self, now: float) -> None:
        """Backtrack toward pre-sign position after misleading sign."""
        dx = self._pre_sign_x - self._x
        dy = self._pre_sign_y - self._y
        dist = math.hypot(dx, dy)

        if dist < 0.3 or now - self._sign_drive_start > 5.0:
            # Reached backtrack target or timeout
            self.get_logger().info("Backtrack complete - resuming exploration")
            self._transition(State.EXPLORING)
            return

        # Reverse toward pre-sign position
        if now - self._sign_drive_start < 2.0:
            # Phase 1: reverse
            self._publish_vel(-0.15, 0.0)
        else:
            # Phase 2: turn toward pre-sign position
            target_yaw = math.atan2(dy, dx)
            error = _normalize_angle(target_yaw - self._yaw)
            angular = _clamp(error * 1.5, -MAX_ANGULAR, MAX_ANGULAR)
            self._publish_vel(MAX_LINEAR * 0.5, angular)

    # ------------------------------------------------------------------
    #  GOAL_SEEK
    # ------------------------------------------------------------------

    def _do_goal_seek(self) -> None:
        if self._regions.front < 0.3:
            self.get_logger().info("MISSION COMPLETE - Goal zone reached!")
            self._transition(State.MISSION_COMPLETE)
            return
        self._publish_vel(MAX_LINEAR, 0.0)

    # ------------------------------------------------------------------
    #  RECOVERY (escalating strategies)
    # ------------------------------------------------------------------

    def _setup_recovery(self) -> None:
        self._recovery_start = time.monotonic()
        # Check if we're stuck in the same area as last recovery
        dist_from_last = math.hypot(
            self._x - self._last_recovery_x,
            self._y - self._last_recovery_y)
        if dist_from_last < 1.0:
            self._consecutive_stucks += 1
        else:
            self._consecutive_stucks = 1
        self._last_recovery_x = self._x
        self._last_recovery_y = self._y
        # Pick a random turn direction for escape
        import random
        turn_dir = random.choice([-1, 1])
        self._recovery_target_yaw = _normalize_angle(
            self._yaw + turn_dir * math.pi / 2)
        self.get_logger().info(
            f"Recovery #{self._consecutive_stucks} at "
            f"({self._x:.1f},{self._y:.1f})")

    def _do_recovery(self) -> None:
        elapsed = time.monotonic() - self._recovery_start

        if self._consecutive_stucks >= 3:
            # PANIC mode: aggressive escape
            self._do_recovery_panic(elapsed)
        else:
            strategy = self._recovery_index % 4
            if strategy == 0:
                self._do_recovery_spin(elapsed)
            elif strategy == 1:
                self._do_recovery_backtrack(elapsed)
            elif strategy == 2:
                self._do_recovery_escape(elapsed)
            else:
                self._do_recovery_reorient(elapsed)

    def _do_recovery_spin(self, elapsed: float) -> None:
        """Strategy 1: Spin in place (~360 deg)."""
        if elapsed > 6.5:
            self._finish_recovery()
            return
        self._publish_vel(0.0, 0.5)

    def _do_recovery_backtrack(self, elapsed: float) -> None:
        """Strategy 2: Backtrack then turn."""
        if elapsed < 2.0:
            self._publish_vel(-0.15, 0.0)
        elif elapsed < 4.0:
            error = _normalize_angle(
                self._recovery_target_yaw - self._yaw)
            angular = _clamp(error * 1.5, -MAX_ANGULAR, MAX_ANGULAR)
            self._publish_vel(0.0, angular)
        else:
            self._finish_recovery()

    def _do_recovery_escape(self, elapsed: float) -> None:
        """Strategy 3: Reverse then turn 90 deg."""
        if elapsed < 3.3:
            self._publish_vel(-0.15, 0.0)
        elif elapsed < 7.0:
            error = _normalize_angle(
                self._recovery_target_yaw - self._yaw)
            if abs(error) < 0.2:
                self._finish_recovery()
                return
            angular = _clamp(error * 1.5, -MAX_ANGULAR, MAX_ANGULAR)
            self._publish_vel(0.0, angular)
        else:
            self._finish_recovery()

    def _do_recovery_reorient(self, elapsed: float) -> None:
        """Strategy 4: Turn toward longest open LiDAR direction."""
        if elapsed > 5.0:
            self._finish_recovery()
            return
        r = self._regions
        directions = {
            'left': (r.left, MAX_ANGULAR),
            'fleft': (r.fleft, MAX_ANGULAR * 0.6),
            'front': (r.front, 0.0),
            'fright': (r.fright, -MAX_ANGULAR * 0.6),
            'right': (r.right, -MAX_ANGULAR),
        }
        best_dir = max(directions.items(), key=lambda d: d[1][0])
        best_range, best_angular = best_dir[1]
        if best_range > WALL_FAR:
            if abs(best_angular) < 0.1:
                self._publish_vel(MAX_LINEAR, 0.0)
            else:
                self._publish_vel(0.05, best_angular)
        else:
            self._publish_vel(0.0, MAX_ANGULAR * 0.5)

    def _do_recovery_panic(self, elapsed: float) -> None:
        """PANIC: Aggressive escape after 3+ consecutive stucks.

        Phase 1 (0-2s): Hard reverse
        Phase 2 (2-5s): Turn ~180 degrees (using saved target yaw)
        Phase 3 (5-8s): Drive forward into new area
        """
        if elapsed < 2.0:
            self._publish_vel(-MAX_LINEAR, 0.0)
        elif elapsed < 5.0:
            # Use the target yaw saved at recovery start, NOT a live
            # recomputation.  The old code recomputed yaw+pi every tick,
            # so the error was always pi and the robot just spun.
            target = _normalize_angle(
                self._recovery_target_yaw + math.pi)
            error = _normalize_angle(target - self._yaw)
            if abs(error) < 0.2:
                # Close enough — skip to drive phase
                self._publish_vel(0.0, 0.0)
            else:
                angular = _clamp(error * 2.0, -MAX_ANGULAR, MAX_ANGULAR)
                self._publish_vel(0.0, angular)
        elif elapsed < 8.0:
            # Drive forward, dodge walls
            if self._regions.front < FRONT_STOP:
                self._publish_vel(0.0, MAX_ANGULAR)
            else:
                self._publish_vel(MAX_LINEAR, 0.0)
        else:
            self.get_logger().warn("PANIC escape complete")
            self._consecutive_stucks = 0
            self._finish_recovery()

    def _find_least_visited_direction(self) -> float:
        """Return angular velocity toward the least-visited neighbouring area.

        Checks the 4 cardinal neighbours of the current cell and picks
        the direction with the lowest visit count that also has open
        LiDAR space.
        """
        cx, cy = _pos_to_cell(self._x, self._y)
        candidates = [
            ((cx + 1, cy), 0.0),      # east  → front
            ((cx - 1, cy), math.pi),   # west  → behind
            ((cx, cy + 1), math.pi / 2),   # north → left
            ((cx, cy - 1), -math.pi / 2),  # south → right
        ]
        best_ang = 0.0
        best_score = float('inf')
        for cell, heading_offset in candidates:
            visits = self._cell_visit_count.get(cell, 0)
            if visits < best_score:
                best_score = visits
                target_yaw = _normalize_angle(self._yaw + heading_offset)
                error = _normalize_angle(target_yaw - self._yaw)
                best_ang = _clamp(error, -MAX_ANGULAR, MAX_ANGULAR)
        return best_ang

    def _finish_recovery(self) -> None:
        self._recovery_index += 1
        self._last_move_time = time.monotonic()
        self._prev_wall_error = 0.0
        self._wall_integral_error = 0.0
        # Flip wall-following side to explore differently
        old = self._wall_follow_side
        self._wall_follow_side = "left" if old == "right" else "right"
        self._override_flipped_cell = None  # allow override to flip again
        self.get_logger().info(
            f"Recovery done: wall-follow {old} -> {self._wall_follow_side}")
        # POST-RECOVERY ESCAPE: drive toward gaps for 5 seconds
        self._post_recovery_until = time.monotonic() + 5.0
        self.get_logger().info("Post-recovery gap drive: 5s")
        # DO NOT clear position history — loop detector needs continuity.
        # Only reset the record timer so we don't get a stale gap.
        self._last_record_time = time.monotonic()
        # DECAY visited-cell counts by 50% instead of wiping.
        # This preserves memory of heavily-visited areas while giving
        # the robot a chance to revisit lightly-visited cells.
        for cell in list(self._cell_visit_count):
            self._cell_visit_count[cell] = max(
                1, self._cell_visit_count[cell] // 2)
        self._transition(State.EXPLORING)

    # ------------------------------------------------------------------
    #  Loop detection (position-history based)
    # ------------------------------------------------------------------

    def _detect_loop(self) -> bool:
        """Check if the robot is near a position it occupied >60 s ago."""
        now = time.monotonic()
        for record in self._position_history:
            age = now - record.timestamp
            if age < LOOP_TIME_THRESHOLD:
                continue
            dist = math.hypot(self._x - record.x, self._y - record.y)
            if dist < LOOP_REVISIT_DIST:
                return True
        return False

    # ------------------------------------------------------------------
    #  Velocity publishing
    # ------------------------------------------------------------------

    def _publish_vel(self, linear: float, angular: float) -> None:
        twist = Twist()
        twist.linear.x = _clamp(linear, -MAX_LINEAR, MAX_LINEAR)
        twist.angular.z = _clamp(angular, -MAX_ANGULAR, MAX_ANGULAR)
        self._cmd_pub.publish(twist)


# ---------------------------------------------------------------------------
#  Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
