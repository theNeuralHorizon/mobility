"""Mission controller for UGV Autonomous Navigation Challenge.

Simple, robust state machine:
- EXPLORING: wall-follow + sign-follow + ArUco detection
- SIGN_FOLLOW: execute sign command (LEFT/RIGHT/FORWARD/STOP/ROTATE/GOAL)
- GOAL_SEEK: drive to goal when all 4 markers found + GOAL sign seen
- RECOVERY: spin + reverse when stuck
- MISSION_COMPLETE: stop

The robot follows the RIGHT wall by default. After getting stuck,
it switches to LEFT wall. This alternation covers the full maze.
"""

from __future__ import annotations

import math
import time
from enum import Enum, auto
from typing import Final

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray, String


# ── Constants ─────────────────────────────────────────────────────────────

MAX_LIN: Final = 0.30        # m/s forward
MAX_ANG: Final = 0.7         # rad/s turning
WALL_TARGET: Final = 0.5     # target distance from wall
STUCK_SEC: Final = 20.0      # seconds before stuck
MARKERS: Final = frozenset({0, 1, 2, 3})


class State(Enum):
    EXPLORING = auto()
    SIGN_FOLLOW = auto()
    GOAL_SEEK = auto()
    RECOVERY = auto()
    DONE = auto()


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y**2 + q.z**2))


def _norm(a: float) -> float:
    while a > math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


class MissionController(Node):
    def __init__(self):
        super().__init__("mission_controller")

        self._state = State.EXPLORING
        self._markers: set[int] = set()
        self._sign: str | None = None

        # Odom
        self._x = self._y = self._yaw_val = 0.0

        # LiDAR regions (right, fright, front, fleft, left)
        self._R = self._FR = self._F = self._FL = self._L = 10.0

        # Stuck
        self._last_move_t = self._start_t = time.monotonic()
        self._last_mx = self._last_my = 0.0

        # Wall-follow direction
        self._follow_left = False
        self._recovery_count = 0

        # Sign follow
        self._sign_yaw = 0.0
        self._sign_phase = ""
        self._sign_t = 0.0

        # Recovery
        self._rec_t = 0.0
        self._rec_yaw = 0.0

        # Logging
        self._log_t = 0.0

        # ROS
        self.create_subscription(LaserScan, "/r1_mini/lidar", self._on_lidar, 10)
        self.create_subscription(Odometry, "/r1_mini/odom", self._on_odom, 10)
        self.create_subscription(Int32MultiArray, "/ugv/aruco/detections", self._on_aruco, 10)
        self.create_subscription(String, "/ugv/sign/direction", self._on_sign, 10)
        self._cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self._state_pub = self.create_publisher(String, "/ugv/mission/state", 10)
        self.create_timer(0.1, self._tick)
        self.get_logger().info("Mission controller started — EXPLORING")

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _on_lidar(self, msg: LaserScan):
        r = list(msg.ranges)
        n = len(r)
        if n == 0:
            return

        def sector(lo, hi):
            lo, hi = max(0, lo), min(n - 1, hi)
            vals = [v for v in r[lo:hi+1] if 0.01 < v < 30 and math.isfinite(v)]
            return min(vals) if vals else 10.0

        self._R = sector(70, 110)
        self._FR = sector(110, 150)
        self._F = sector(150, 210)
        self._FL = sector(210, 250)
        self._L = sector(250, 290)

    def _on_odom(self, msg: Odometry):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        self._yaw_val = _yaw(msg.pose.pose.orientation)

        d = math.hypot(self._x - self._last_mx, self._y - self._last_my)
        if d > 0.15:
            self._last_move_t = time.monotonic()
            self._last_mx, self._last_my = self._x, self._y

    def _on_aruco(self, msg: Int32MultiArray):
        for mid in msg.data:
            if mid in MARKERS and mid not in self._markers:
                self._markers.add(mid)
                self.get_logger().info(f">>> ArUco {mid} found! ({len(self._markers)}/4)")
                if self._markers >= MARKERS:
                    self.get_logger().info(">>> ALL 4 MARKERS COLLECTED!")

    def _on_sign(self, msg: String):
        if self._state not in (State.EXPLORING,):
            return
        d = msg.data
        self.get_logger().info(f">>> Sign detected: {d}")

        if d == "GOAL":
            if self._markers >= MARKERS:
                self.get_logger().info(">>> GOAL sign + all markers → GOAL_SEEK")
                self._set_state(State.GOAL_SEEK)
            return

        # Validate: don't turn into a wall
        if d == "LEFT" and self._L < 0.5:
            self.get_logger().warn("LEFT sign rejected (wall)")
            return
        if d == "RIGHT" and self._R < 0.5:
            self.get_logger().warn("RIGHT sign rejected (wall)")
            return

        self._sign = d
        self._set_state(State.SIGN_FOLLOW)

    # ── State machine ─────────────────────────────────────────────────────

    def _set_state(self, s: State):
        if s == self._state:
            return
        old = self._state
        self._state = s
        self.get_logger().info(f"State: {old.name} → {s.name}")

        if s == State.SIGN_FOLLOW:
            self._init_sign()
        elif s == State.RECOVERY:
            self._rec_t = time.monotonic()
            self._rec_yaw = _norm(self._yaw_val + math.pi)

        m = String()
        m.data = s.name
        self._state_pub.publish(m)

    def _tick(self):
        now = time.monotonic()

        # Log position every 20s
        if now - self._log_t > 20:
            side = "L" if self._follow_left else "R"
            self.get_logger().info(
                f"pos=({self._x:.1f},{self._y:.1f}) markers={len(self._markers)}/4 "
                f"wall={side} F={self._F:.2f} R={self._R:.2f} L={self._L:.2f}")
            self._log_t = now

        if self._state == State.DONE:
            self._vel(0, 0)
            return

        # Stuck check (not during recovery)
        if self._state != State.RECOVERY:
            if now - self._last_move_t > STUCK_SEC:
                self.get_logger().warn(f"Stuck {STUCK_SEC}s → RECOVERY")
                self._set_state(State.RECOVERY)
                return

        if self._state == State.EXPLORING:
            self._do_explore()
        elif self._state == State.SIGN_FOLLOW:
            self._do_sign()
        elif self._state == State.GOAL_SEEK:
            self._do_goal()
        elif self._state == State.RECOVERY:
            self._do_recover(now)

    # ── EXPLORING ─────────────────────────────────────────────────────────

    def _do_explore(self):
        """Wall-following with left/right alternation."""
        if self._follow_left:
            self._wall_follow_left()
        else:
            self._wall_follow_right()

    def _wall_follow_right(self):
        """Follow the right wall."""
        F, FR, R = self._F, self._FR, self._R

        if F < 0.4:
            # Wall ahead — turn left
            self._vel(0.05, MAX_ANG * 0.8)
        elif FR < 0.3:
            # Too close to front-right — veer left
            self._vel(MAX_LIN * 0.6, 0.3)
        elif R > 1.0 and FR > 1.0:
            # Right side open — turn right to find wall
            self._vel(MAX_LIN * 0.9, -0.4)
        elif R < 10:
            # PD wall follow
            err = WALL_TARGET - R
            ang = _clamp(0.8 * err, -MAX_ANG, MAX_ANG)
            self._vel(MAX_LIN, ang)
        else:
            # No wall — go forward, drift right
            self._vel(MAX_LIN, -0.2)

    def _wall_follow_left(self):
        """Follow the left wall (mirror of right)."""
        F, FL, L = self._F, self._FL, self._L

        if F < 0.4:
            self._vel(0.05, -MAX_ANG * 0.8)
        elif FL < 0.3:
            self._vel(MAX_LIN * 0.6, -0.3)
        elif L > 1.0 and FL > 1.0:
            self._vel(MAX_LIN * 0.9, 0.4)
        elif L < 10:
            err = WALL_TARGET - L
            ang = _clamp(0.8 * err, -MAX_ANG, MAX_ANG)
            self._vel(MAX_LIN, -ang)
        else:
            self._vel(MAX_LIN, 0.2)

    # ── SIGN_FOLLOW ───────────────────────────────────────────────────────

    def _init_sign(self):
        s = self._sign
        if s == "LEFT":
            self._sign_yaw = _norm(self._yaw_val + math.pi / 2)
            self._sign_phase = "turn"
        elif s == "RIGHT":
            self._sign_yaw = _norm(self._yaw_val - math.pi / 2)
            self._sign_phase = "turn"
        elif s == "FORWARD":
            self._sign_phase = "drive"
            self._sign_t = time.monotonic()
        elif s == "STOP":
            self._sign_phase = "stop"
            self._sign_t = time.monotonic()
        elif s == "INPLACE_ROTATION":
            self._sign_phase = "spin"
            self._sign_t = time.monotonic()
        else:
            self._set_state(State.EXPLORING)

    def _do_sign(self):
        now = time.monotonic()

        if self._sign_phase == "turn":
            err = _norm(self._sign_yaw - self._yaw_val)
            if abs(err) < 0.12:
                self._sign_phase = "drive"
                self._sign_t = now
            else:
                self._vel(0, _clamp(err * 2.0, -MAX_ANG, MAX_ANG))

        elif self._sign_phase == "drive":
            if now - self._sign_t > 5.0 or self._F < 0.4:
                self._set_state(State.EXPLORING)
            else:
                self._vel(MAX_LIN, 0)

        elif self._sign_phase == "stop":
            if now - self._sign_t > 2.0:
                self._set_state(State.EXPLORING)
            else:
                self._vel(0, 0)

        elif self._sign_phase == "spin":
            if now - self._sign_t > 6.0:
                self._set_state(State.EXPLORING)
            else:
                self._vel(0, MAX_ANG)

    # ── GOAL_SEEK ─────────────────────────────────────────────────────────

    def _do_goal(self):
        if self._F < 0.3:
            self.get_logger().info("★ MISSION COMPLETE ★")
            self._set_state(State.DONE)
        else:
            self._vel(MAX_LIN, 0)

    # ── RECOVERY ──────────────────────────────────────────────────────────

    def _do_recover(self, now: float):
        elapsed = now - self._rec_t
        strategy = self._recovery_count % 3

        if strategy == 0:
            # Spin 360°
            if elapsed > 7:
                self._end_recovery()
            else:
                self._vel(0, 0.6)
        elif strategy == 1:
            # Reverse
            if elapsed > 3:
                self._end_recovery()
            else:
                self._vel(-0.2, 0)
        else:
            # Reverse + turn 180°
            if elapsed < 2:
                self._vel(-0.2, 0)
            elif elapsed < 6:
                err = _norm(self._rec_yaw - self._yaw_val)
                if abs(err) < 0.2:
                    self._end_recovery()
                else:
                    self._vel(0, _clamp(err * 1.5, -MAX_ANG, MAX_ANG))
            else:
                self._end_recovery()

    def _end_recovery(self):
        self._recovery_count += 1
        self._follow_left = not self._follow_left
        side = "LEFT" if self._follow_left else "RIGHT"
        self.get_logger().info(f"Recovery done → follow {side} wall")
        self._last_move_t = time.monotonic()
        self._set_state(State.EXPLORING)

    # ── Velocity ──────────────────────────────────────────────────────────

    def _vel(self, lin: float, ang: float):
        t = Twist()
        t.linear.x = _clamp(lin, -MAX_LIN, MAX_LIN)
        t.angular.z = _clamp(ang, -MAX_ANG, MAX_ANG)
        self._cmd.publish(t)


def main(args=None):
    rclpy.init(args=args)
    n = MissionController()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
