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
KD: Final[float] = 0.5

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
) -> tuple[float, float, float]:
    """Compute (linear, angular, new_prev_error) using PD wall-follower.

    Returns the raw command from the right-wall-following algorithm.
    """
    r = regions

    if r.front < FRONT_STOP:
        return 0.0, MAX_ANGULAR, prev_error

    if r.front < FRONT_SLOW:
        return 0.08, MAX_ANGULAR * 0.6, prev_error

    if r.fright < WALL_CLOSE:
        return MAX_LINEAR * 0.7, 0.3, prev_error

    if r.right > WALL_FAR and r.fright > WALL_FAR:
        return MAX_LINEAR, -0.15, prev_error

    if r.right < SAFE_RANGE_MAX:
        error = WALL_DIST - r.right
        d_error = error - prev_error
        angular = _clamp(KP * error + KD * d_error, -MAX_ANGULAR, MAX_ANGULAR)
        return MAX_LINEAR, angular, error

    return MAX_LINEAR, -0.2, prev_error


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

        # -- visited-cell tracking --
        self._visited_cells: set[tuple[int, int]] = set()
        self._cell_visit_count: dict[tuple[int, int], int] = defaultdict(int)
        self._last_cell: tuple[int, int] = (0, 0)

        # -- loop detection (position history) --
        self._position_history: list[PositionRecord] = []
        self._last_record_time: float = 0.0

        # -- recovery --
        self._recovery_index: int = 0
        self._recovery_start: float = 0.0
        self._recovery_target_yaw: float = 0.0

        # -- sign follow --
        self._sign_target_yaw: float = 0.0
        self._sign_phase: str = "turning"
        self._sign_drive_start: float = 0.0

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
        if direction == "LEFT" and self._regions.left < 0.4:
            self.get_logger().warn("LEFT sign rejected - wall on left")
            return False
        if direction == "RIGHT" and self._regions.right < 0.4:
            self.get_logger().warn("RIGHT sign rejected - wall on right")
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
        cell = _pos_to_cell(self._x, self._y)
        visits = self._cell_visit_count.get(cell, 0)

        # Override wall-follower when cell is revisited too often
        if visits >= REVISIT_LIMIT:
            self._do_exploring_override()
            return

        linear, angular, new_error = _wall_follow_cmd(
            self._regions, self._prev_wall_error)
        self._prev_wall_error = new_error
        self._publish_vel(linear, angular)

    def _do_exploring_override(self) -> None:
        """Override wall-follower when current cell revisited too often.

        Strategy: if the normal wall-follower would turn right (or go
        straight), force a left turn instead.  If already turning left,
        go straight.  This breaks the repetitive loop pattern.
        """
        r = self._regions

        if r.front < FRONT_STOP:
            # Still need to avoid frontal collision - turn left hard
            self._publish_vel(0.0, MAX_ANGULAR)
            return

        # Prefer going toward the least-visited adjacent direction
        if r.left > WALL_CLOSE and r.fleft > WALL_CLOSE:
            # Left is open - override: turn left
            self.get_logger().info(
                "Cell revisit override: turning LEFT to break loop")
            self._publish_vel(MAX_LINEAR * 0.8, 0.3)
        elif r.front > FRONT_SLOW:
            # Straight is open - just go forward
            self.get_logger().info(
                "Cell revisit override: going STRAIGHT to break loop")
            self._publish_vel(MAX_LINEAR, 0.0)
        else:
            # Fallback: slow left turn
            self._publish_vel(0.05, MAX_ANGULAR * 0.7)

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
        if now - self._sign_drive_start > 2.0 or self._regions.front < 0.4:
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
    #  RECOVERY (3 strategies, cycled)
    # ------------------------------------------------------------------

    def _setup_recovery(self) -> None:
        self._recovery_start = time.monotonic()
        self._recovery_target_yaw = _normalize_angle(self._yaw + math.pi / 2)

    def _do_recovery(self) -> None:
        elapsed = time.monotonic() - self._recovery_start
        strategy = self._recovery_index % 4

        if strategy == 0:
            self._do_recovery_spin(elapsed)
        elif strategy == 1:
            self._do_recovery_backtrack(elapsed)
        elif strategy == 3:
            self._do_recovery_reorient(elapsed)
        elif strategy == 2:
            self._do_recovery_escape(elapsed)

    def _do_recovery_spin(self, elapsed: float) -> None:
        """Strategy 1: Spin in place (~360 deg at 0.5 rad/s)."""
        if elapsed > 6.5:
            self._finish_recovery()
            return
        self._publish_vel(0.0, 0.5)

    def _do_recovery_backtrack(self, elapsed: float) -> None:
        """Strategy 2: Backtrack 0.3 m at -0.15 m/s."""
        if elapsed > 2.0:
            self._finish_recovery()
            return
        self._publish_vel(-0.15, 0.0)

    def _do_recovery_escape(self, elapsed: float) -> None:
        """Strategy 3: Reverse 0.5 m then turn 90 deg."""
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
        """Strategy 4: Turn toward the longest open LiDAR direction."""
        if elapsed > 5.0:
            self._finish_recovery()
            return
        r = self._regions
        # Find the direction with the most open space
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
            # Open space found — turn toward it then drive
            if abs(best_angular) < 0.1:
                self._publish_vel(MAX_LINEAR, 0.0)
            else:
                self._publish_vel(0.05, best_angular)
        else:
            # No clear opening — spin slowly
            self._publish_vel(0.0, MAX_ANGULAR * 0.5)

    def _finish_recovery(self) -> None:
        self._recovery_index += 1
        self._last_move_time = time.monotonic()
        self._prev_wall_error = 0.0
        # Clear history so loop detection does not immediately re-trigger
        self._position_history.clear()
        self._last_record_time = time.monotonic()
        # Clear visited cells for fresh exploration after recovery
        self._visited_cells.clear()
        self._cell_visit_count.clear()
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
