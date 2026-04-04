"""Mission controller node for UGV Autonomous Navigation Challenge.

Right-wall-following state machine that navigates a map-less arena,
detects ArUco markers, interprets directional signs, and reaches the goal.

Uses a PD controller to maintain a fixed distance from the right wall,
with 5-region LiDAR decomposition for decision making.

States: EXPLORING, SIGN_FOLLOW, MARKER_APPROACH, GOAL_SEEK, RECOVERY, MISSION_COMPLETE

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
STUCK_TIMEOUT: Final[float] = 10.0
STUCK_MOVE_THRESHOLD: Final[float] = 0.1
LOOP_REVISIT_DIST: Final[float] = 0.8
LOOP_TIME_THRESHOLD: Final[float] = 60.0
REQUIRED_MARKERS: Final[frozenset[int]] = frozenset({0, 1, 2, 3})
SAFE_RANGE_MAX: Final[float] = 10.0


# ---------------------------------------------------------------------------
#  Enums / Data
# ---------------------------------------------------------------------------

class State(Enum):
    EXPLORING = auto()
    SIGN_FOLLOW = auto()
    MARKER_APPROACH = auto()
    GOAL_SEEK = auto()
    RECOVERY = auto()
    MISSION_COMPLETE = auto()


@dataclass(frozen=True)
class PositionRecord:
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
    """Return min of *finite* values, or *default* if none are valid."""
    finite = [v for v in values if math.isfinite(v) and v > 0.01]
    return min(min(finite), default) if finite else default


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


# ---------------------------------------------------------------------------
#  Node
# ---------------------------------------------------------------------------

class MissionController(Node):
    """Right-wall-following mission controller."""

    def __init__(self) -> None:
        super().__init__("mission_controller")

        # ----- state machine -----
        self._state: State = State.EXPLORING
        self._prev_state: State = State.EXPLORING

        # ----- perception data -----
        self._regions: LidarRegions = LidarRegions()
        self._visited_markers: set[int] = set()
        self._current_sign: str | None = None

        # ----- odometry -----
        self._x: float = 0.0
        self._y: float = 0.0
        self._yaw: float = 0.0

        # ----- stuck detection -----
        self._last_move_time: float = time.monotonic()
        self._last_move_x: float = 0.0
        self._last_move_y: float = 0.0

        # ----- PD wall-follow state -----
        self._prev_wall_error: float = 0.0

        # ----- loop detection -----
        self._position_history: list[PositionRecord] = []
        self._last_record_time: float = 0.0

        # ----- recovery -----
        self._recovery_index: int = 0
        self._recovery_start: float = 0.0
        self._recovery_target_yaw: float = 0.0

        # ----- sign follow -----
        self._sign_target_yaw: float = 0.0
        self._sign_phase: str = "turning"
        self._sign_drive_start: float = 0.0

        # ----- ROS2 interfaces -----
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

        dx = self._x - self._last_move_x
        dy = self._y - self._last_move_y
        if math.hypot(dx, dy) > STUCK_MOVE_THRESHOLD:
            self._last_move_time = time.monotonic()
            self._last_move_x = self._x
            self._last_move_y = self._y

        now = time.monotonic()
        if now - self._last_record_time > 2.0:
            self._position_history.append(
                PositionRecord(self._x, self._y, now))
            self._last_record_time = now
            cutoff = now - 120.0
            self._position_history = [
                p for p in self._position_history if p.timestamp > cutoff
            ]

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

        # Reject misleading signs using LiDAR cross-check
        if direction == "LEFT" and self._regions.left < 0.4:
            self.get_logger().warn("LEFT sign rejected - wall on left")
            return
        if direction == "RIGHT" and self._regions.right < 0.4:
            self.get_logger().warn("RIGHT sign rejected - wall on right")
            return

        self._current_sign = direction
        self._transition(State.SIGN_FOLLOW)

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

        # Stuck detection (skip during recovery)
        if self._state != State.RECOVERY:
            stuck_secs = time.monotonic() - self._last_move_time
            if stuck_secs > STUCK_TIMEOUT:
                self.get_logger().warn(
                    f"Stuck for {stuck_secs:.1f}s - entering RECOVERY")
                self._transition(State.RECOVERY)
                return

        # Loop detection during exploration
        if self._state == State.EXPLORING and self._detect_loop():
            self.get_logger().warn("Loop detected - entering RECOVERY")
            self._transition(State.RECOVERY)
            return

        # Dispatch
        handlers = {
            State.EXPLORING: self._do_exploring,
            State.SIGN_FOLLOW: self._do_sign_follow,
            State.GOAL_SEEK: self._do_goal_seek,
            State.RECOVERY: self._do_recovery,
        }
        handler = handlers.get(self._state)
        if handler is not None:
            handler()

    # ------------------------------------------------------------------
    #  EXPLORING: Right-wall following with PD controller
    # ------------------------------------------------------------------

    def _do_exploring(self) -> None:
        r = self._regions

        # Determine case
        all_clear = (r.front > 1.0 and r.fright > 1.0
                     and r.right > 1.0 and r.fleft > 1.0 and r.left > 1.0)
        front_blocked = r.front < 0.5
        right_found = r.right < 1.0

        if all_clear:
            # Case 1: Nothing nearby - turn right gently to find wall
            linear = MAX_LINEAR
            angular = -0.3
        elif not front_blocked and right_found:
            # Case 2: Front clear, right wall found - PD wall follow
            error = WALL_DIST - r.right
            d_error = error - self._prev_wall_error
            self._prev_wall_error = error
            angular = _clamp(KP * error + KD * d_error,
                             -MAX_ANGULAR, MAX_ANGULAR)
            linear = MAX_LINEAR
        elif front_blocked and r.right < 0.5 and r.left < 0.5:
            # Case 5: Corridor dead-end - turn left
            linear = 0.0
            angular = MAX_ANGULAR
        elif front_blocked and r.right < 0.5:
            # Case 4: Front + right blocked - sharp left turn
            linear = 0.0
            angular = MAX_ANGULAR
        elif front_blocked:
            # Case 3: Front blocked - turn left
            linear = 0.05
            angular = MAX_ANGULAR * 0.7
        else:
            # Default: gentle right-wall follow
            error = WALL_DIST - r.right
            d_error = error - self._prev_wall_error
            self._prev_wall_error = error
            angular = _clamp(KP * error + KD * d_error,
                             -MAX_ANGULAR, MAX_ANGULAR)
            linear = MAX_LINEAR * 0.8

        # Slow down in tight spaces
        min_clearance = min(r.left, r.right, r.front)
        if min_clearance < 0.3:
            linear *= 0.5

        self._publish_vel(linear, angular)

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
            error = _normalize_angle(self._sign_target_yaw - self._yaw)
            if abs(error) < 0.15:
                self._sign_phase = "driving"
                self._sign_drive_start = now
                self._publish_vel(0.0, 0.0)
            else:
                angular = _clamp(error * 1.5, -MAX_ANGULAR, MAX_ANGULAR)
                self._publish_vel(0.0, angular)

        elif self._sign_phase == "driving":
            if now - self._sign_drive_start > 2.0 or self._regions.front < 0.4:
                self._transition(State.EXPLORING)
                return
            self._publish_vel(MAX_LINEAR, 0.0)

        elif self._sign_phase == "stopped":
            if now - self._sign_drive_start > 2.0:
                self._transition(State.EXPLORING)
                return
            self._publish_vel(0.0, 0.0)

        elif self._sign_phase == "spinning":
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
        strategy = self._recovery_index % 3

        if strategy == 0:
            # Strategy 1: Spin in place (360 degrees at 0.5 rad/s ~ 12.6s)
            if elapsed > 6.5:
                self._finish_recovery()
                return
            self._publish_vel(0.0, 0.5)

        elif strategy == 1:
            # Strategy 2: Backtrack 0.3m at -0.15 m/s ~ 2s
            if elapsed > 2.0:
                self._finish_recovery()
                return
            self._publish_vel(-0.15, 0.0)

        elif strategy == 2:
            # Strategy 3: Escape dead-end - reverse 0.5m + turn 180
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

    def _finish_recovery(self) -> None:
        self._recovery_index += 1
        self._last_move_time = time.monotonic()
        self._prev_wall_error = 0.0
        self._transition(State.EXPLORING)

    # ------------------------------------------------------------------
    #  Loop detection
    # ------------------------------------------------------------------

    def _detect_loop(self) -> bool:
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
