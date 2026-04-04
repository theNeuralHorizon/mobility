"""Mission controller node for UGV Autonomous Navigation Challenge.

State-machine brain that navigates the robot through a map-less arena,
detects ArUco markers, interprets directional signs, and reaches the goal.

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

import math
import time
from dataclasses import dataclass, field
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray, String


class State(Enum):
    EXPLORING = auto()
    SIGN_FOLLOW = auto()
    MARKER_APPROACH = auto()
    GOAL_SEEK = auto()
    RECOVERY = auto()
    MISSION_COMPLETE = auto()


@dataclass
class PositionRecord:
    x: float
    y: float
    timestamp: float


MAX_LINEAR_VEL = 0.3
MAX_ANGULAR_VEL = 0.8
WALL_FOLLOW_DIST = 0.4
STUCK_TIMEOUT = 8.0
LOOP_REVISIT_DIST = 0.8
LOOP_TIME_THRESHOLD = 60.0
REQUIRED_MARKERS = {0, 1, 2, 3}


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _yaw_from_quaternion(q) -> float:
    """Extract yaw from quaternion."""
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


class MissionController(Node):

    def __init__(self) -> None:
        super().__init__("mission_controller")

        # State
        self._state = State.EXPLORING
        self._prev_state = State.EXPLORING
        self._visited_markers: set[int] = set()
        self._current_sign: str | None = None

        # Position tracking
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._last_move_time = time.monotonic()
        self._last_x = 0.0
        self._last_y = 0.0

        # Loop detection
        self._position_history: list[PositionRecord] = []
        self._last_record_time = 0.0

        # Recovery
        self._recovery_index = 0
        self._recovery_start_time = 0.0
        self._recovery_target_yaw = 0.0

        # Sign follow
        self._sign_target_yaw = 0.0
        self._sign_phase = "turning"  # "turning" or "driving"
        self._sign_drive_start = 0.0

        # LiDAR data
        self._front_dist = float("inf")
        self._left_dist = float("inf")
        self._right_dist = float("inf")
        self._front_left_dist = float("inf")
        self._front_right_dist = float("inf")

        # Subscribers
        self.create_subscription(
            LaserScan, "/r1_mini/lidar", self._lidar_cb, 10)
        self.create_subscription(
            Odometry, "/r1_mini/odom", self._odom_cb, 10)
        self.create_subscription(
            Int32MultiArray, "/ugv/aruco/detections", self._aruco_cb, 10)
        self.create_subscription(
            String, "/ugv/sign/direction", self._sign_cb, 10)

        # Publishers
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._state_pub = self.create_publisher(
            String, "/ugv/mission/state", 10)

        # Control loop at 10 Hz
        self.create_timer(0.1, self._control_loop)

        self.get_logger().info("Mission controller started - state: EXPLORING")

    # ---- Callbacks --------------------------------------------------------

    def _lidar_cb(self, msg: LaserScan) -> None:
        """Extract distances from LiDAR scan sectors.

        Scan layout: 360 rays, angle_min=-pi (behind), angle_max=+pi (behind)
        Index 0=-pi(back), 90=-pi/2(right), 180=0(front), 270=+pi/2(left), 359≈pi(back)
        """
        n = len(msg.ranges)
        if n == 0:
            return

        def _angle_to_idx(angle_deg: float) -> int:
            """Convert angle in degrees (0=front, 90=left, -90=right) to index."""
            angle_rad = math.radians(angle_deg)
            idx = int((angle_rad - msg.angle_min) / msg.angle_increment)
            return max(0, min(n - 1, idx))

        def _sector_min(center_deg: float, width_deg: float) -> float:
            lo = _angle_to_idx(center_deg - width_deg / 2)
            hi = _angle_to_idx(center_deg + width_deg / 2)
            if lo > hi:
                lo, hi = hi, lo
            sector = msg.ranges[lo:hi + 1]
            valid = [r for r in sector if msg.range_min < r < msg.range_max]
            return min(valid) if valid else float("inf")

        self._front_dist = _sector_min(0, 30)
        self._front_left_dist = _sector_min(30, 30)
        self._front_right_dist = _sector_min(-30, 30)
        self._left_dist = _sector_min(90, 20)
        self._right_dist = _sector_min(-90, 20)

    def _odom_cb(self, msg: Odometry) -> None:
        """Track robot position and detect if stuck."""
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        self._yaw = _yaw_from_quaternion(msg.pose.pose.orientation)

        # Check movement for stuck detection
        dx = self._x - self._last_x
        dy = self._y - self._last_y
        dist_moved = math.hypot(dx, dy)

        if dist_moved > 0.02:
            self._last_move_time = time.monotonic()
            self._last_x = self._x
            self._last_y = self._y

        # Record position for loop detection
        now = time.monotonic()
        if now - self._last_record_time > 2.0:
            self._position_history.append(
                PositionRecord(self._x, self._y, now))
            self._last_record_time = now
            # Keep last 120 seconds of history
            cutoff = now - 120.0
            self._position_history = [
                p for p in self._position_history if p.timestamp > cutoff
            ]

    def _aruco_cb(self, msg: Int32MultiArray) -> None:
        """Log detected ArUco markers."""
        for mid in msg.data:
            if mid in REQUIRED_MARKERS and mid not in self._visited_markers:
                self._visited_markers.add(mid)
                self.get_logger().info(
                    f"ArUco {mid} visited! ({len(self._visited_markers)}/4)")
                if self._visited_markers == REQUIRED_MARKERS:
                    self.get_logger().info("ALL 4 MARKERS COLLECTED!")

    def _sign_cb(self, msg: String) -> None:
        """Handle detected sign direction."""
        if self._state in (State.RECOVERY, State.MISSION_COMPLETE,
                           State.SIGN_FOLLOW):
            return

        direction = msg.data
        if direction == "GOAL":
            if self._visited_markers == REQUIRED_MARKERS:
                self._transition(State.GOAL_SEEK)
                return
            else:
                self.get_logger().info(
                    "GOAL sign seen but markers incomplete - ignoring")
                return

        # Cross-reference with LiDAR to reject misleading signs
        if direction == "LEFT" and self._left_dist < 0.4:
            self.get_logger().warn("LEFT sign rejected - wall on left")
            return
        if direction == "RIGHT" and self._right_dist < 0.4:
            self.get_logger().warn("RIGHT sign rejected - wall on right")
            return

        self._current_sign = direction
        self._transition(State.SIGN_FOLLOW)

    # ---- State transitions -------------------------------------------------

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

    # ---- Main control loop --------------------------------------------------

    def _control_loop(self) -> None:
        if self._state == State.MISSION_COMPLETE:
            self._publish_vel(0.0, 0.0)
            return

        # Stuck detection (except during recovery)
        if self._state != State.RECOVERY:
            elapsed = time.monotonic() - self._last_move_time
            if elapsed > STUCK_TIMEOUT:
                self.get_logger().warn(
                    f"Stuck for {elapsed:.1f}s - entering RECOVERY")
                self._transition(State.RECOVERY)
                return

        # Loop detection during exploration
        if self._state == State.EXPLORING and self._detect_loop():
            self.get_logger().warn("Loop detected - changing direction")
            self._transition(State.RECOVERY)
            return

        # Dispatch to state handler
        if self._state == State.EXPLORING:
            self._do_exploring()
        elif self._state == State.SIGN_FOLLOW:
            self._do_sign_follow()
        elif self._state == State.GOAL_SEEK:
            self._do_goal_seek()
        elif self._state == State.RECOVERY:
            self._do_recovery()

    # ---- State handlers ------------------------------------------------------

    def _do_exploring(self) -> None:
        """Right-wall following exploration."""
        linear = MAX_LINEAR_VEL
        angular = 0.0

        if self._front_dist < 0.5:
            # Front blocked - turn left
            linear = 0.05
            angular = MAX_ANGULAR_VEL
        elif self._right_dist > 1.5:
            # Right is wide open - turn right to explore
            linear = MAX_LINEAR_VEL * 0.7
            angular = -MAX_ANGULAR_VEL * 0.5
        elif self._right_dist < 0.25:
            # Too close to right wall - steer left
            linear = MAX_LINEAR_VEL * 0.8
            angular = MAX_ANGULAR_VEL * 0.3
        elif self._right_dist > WALL_FOLLOW_DIST + 0.1:
            # Drifting from right wall - steer right
            angular = -0.2
        elif self._right_dist < WALL_FOLLOW_DIST - 0.1:
            # Too close to right wall
            angular = 0.2

        # Slow down in narrow passages
        min_side = min(self._left_dist, self._right_dist)
        if min_side < 0.35:
            linear *= 0.5

        self._publish_vel(linear, angular)

    def _setup_sign_follow(self) -> None:
        """Configure sign-following behavior."""
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
        """Execute the current sign command."""
        sign = self._current_sign
        now = time.monotonic()

        if self._sign_phase == "turning":
            error = _normalize_angle(self._sign_target_yaw - self._yaw)
            if abs(error) < 0.15:
                self._sign_phase = "driving"
                self._sign_drive_start = now
                self._publish_vel(0.0, 0.0)
            else:
                angular = _clamp(error * 1.5, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)
                self._publish_vel(0.0, angular)

        elif self._sign_phase == "driving":
            if now - self._sign_drive_start > 2.0:
                self._transition(State.EXPLORING)
                return
            if self._front_dist < 0.4:
                self._transition(State.EXPLORING)
                return
            self._publish_vel(MAX_LINEAR_VEL, 0.0)

        elif self._sign_phase == "stopped":
            if now - self._sign_drive_start > 2.0:
                self._transition(State.EXPLORING)
                return
            self._publish_vel(0.0, 0.0)

        elif self._sign_phase == "spinning":
            if now - self._sign_drive_start > 5.0:
                self._transition(State.EXPLORING)
                return
            self._publish_vel(0.0, MAX_ANGULAR_VEL)

    def _do_goal_seek(self) -> None:
        """Drive forward toward goal zone."""
        if self._front_dist < 0.3:
            self.get_logger().info("MISSION COMPLETE - Goal zone reached!")
            self._transition(State.MISSION_COMPLETE)
            return
        self._publish_vel(MAX_LINEAR_VEL, 0.0)

    def _setup_recovery(self) -> None:
        """Initialize recovery behavior."""
        self._recovery_start_time = time.monotonic()
        self._recovery_target_yaw = _normalize_angle(
            self._yaw + math.pi)  # For escape

    def _do_recovery(self) -> None:
        """Execute recovery behaviors (rotate, backtrack, escape)."""
        elapsed = time.monotonic() - self._recovery_start_time
        strategy = self._recovery_index % 3

        if strategy == 0:
            # Strategy 1: Rotate in place (360 degrees)
            if elapsed > 4.0:
                self._finish_recovery()
                return
            self._publish_vel(0.0, MAX_ANGULAR_VEL)

        elif strategy == 1:
            # Strategy 2: Backtrack (reverse 0.5m)
            if elapsed > 2.0:
                self._finish_recovery()
                return
            self._publish_vel(-MAX_LINEAR_VEL * 0.5, 0.0)

        elif strategy == 2:
            # Strategy 3: Escape dead-end (reverse + 180 turn)
            if elapsed < 1.5:
                self._publish_vel(-MAX_LINEAR_VEL * 0.5, 0.0)
            elif elapsed < 4.0:
                error = _normalize_angle(
                    self._recovery_target_yaw - self._yaw)
                if abs(error) < 0.2:
                    self._finish_recovery()
                    return
                angular = _clamp(
                    error * 1.5, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)
                self._publish_vel(0.0, angular)
            else:
                self._finish_recovery()

    def _finish_recovery(self) -> None:
        """End recovery and return to exploring."""
        self._recovery_index += 1
        self._last_move_time = time.monotonic()
        self._transition(State.EXPLORING)

    # ---- Loop detection -----------------------------------------------------

    def _detect_loop(self) -> bool:
        """Check if robot has revisited an old position."""
        now = time.monotonic()
        for record in self._position_history:
            age = now - record.timestamp
            if age < LOOP_TIME_THRESHOLD:
                continue
            dist = math.hypot(self._x - record.x, self._y - record.y)
            if dist < LOOP_REVISIT_DIST:
                return True
        return False

    # ---- Velocity publishing ------------------------------------------------

    def _publish_vel(self, linear: float, angular: float) -> None:
        twist = Twist()
        twist.linear.x = _clamp(linear, -MAX_LINEAR_VEL, MAX_LINEAR_VEL)
        twist.angular.z = _clamp(angular, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)
        self._cmd_pub.publish(twist)


def main(args=None):
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
