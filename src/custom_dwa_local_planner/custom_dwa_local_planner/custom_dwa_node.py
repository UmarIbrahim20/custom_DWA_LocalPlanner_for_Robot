#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray


# ============================================================
# DWA CONFIGURATION
# ============================================================

class Config:
    """Defines all static parameters for the DWA planner and robot."""
    def __init__(self):
        # --- Robot Dynamic Limits ---
        self.MAX_VELOCITY = 0.26        # [m/s] Max forward linear velocity
        self.MIN_VELOCITY = 0.01        # [m/s] Min forward linear velocity
        self.MAX_YAWRATE = 2.2          # [rad/s] Max angular velocity

        self.MAX_ACCELERATION = 0.5     # [m/s^2] Max linear acceleration
        self.MAX_DECELERATION = 4.0     # [m/s^2] Max linear deceleration (used when slowing down)
        self.MAX_D_YAWRATE = 3.2        # [rad/s^2] Max angular acceleration/deceleration

        # --- Robot Footprint/Size Parameters ---
        self.ROBOT_LENGTH = 0.280       # [m] Length of the robot body
        self.ROBOT_WIDTH = 0.320        # [m] Width of the robot body
        # Effective radius for simple collision checking (corner-to-center)
        self.ROBOT_RADIUS = math.hypot(self.ROBOT_LENGTH / 2.0, self.ROBOT_WIDTH / 2.0)
        self.FOOTPRINT_PADDING = 0.02   # [m] Extra safety margin around the robot footprint

        # --- Simulation Parameters ---
        self.PREDICT_TIME = 4.0         # [s] Look-ahead time for trajectory prediction
        self.SIM_PERIOD = 0.1           # [s] Time step (dt) for simulation and control loop (10 Hz)

        # --- Sampling Parameters ---
        self.VELOCITY_SAMPLES = 8       # Number of linear velocities to sample in the dynamic window
        self.YAWRATE_SAMPLES = 65       # Number of angular velocities to sample in the dynamic window

        # --- Cost Weightings ---
        self.OBSTACLE_COST_GAIN = 1.2
        self.TO_GOAL_COST_GAIN = 4.0    # Heavy weighting to encourage turning towards the goal
        self.SPEED_COST_GAIN = 7.0      # Heavy weighting to encourage high speed (up to MAX_VELOCITY)
        self.PATH_COST_GAIN = 0.0       # Path cost is disabled

        # --- Perception / Goal Acceptance ---
        self.OBS_RANGE = 3.0            # [m] Max distance for laser points to be considered as obstacles
        self.GOAL_THRESHOLD = 0.3       # [m] Distance within which the goal is considered reached

        # --- Stuck Prevention ---
        self.ROBOT_STUCK_FLAG_CONS = 0.005 # [m/s] Velocity threshold for triggering deadlock escape


# Instantiate the configuration
config = Config()


# ============================================================
# UTILS
# ============================================================

def normalize_angle(angle: float) -> float:
    """Normalize an angle to the range [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def get_yaw_from_quaternion(orientation) -> float:
    """Converts a geometry_msgs/Quaternion to an Euler yaw angle."""
    x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


# ============================================================
# DWA CORE FUNCTIONS
# ============================================================

def calculate_dynamic_window(x: np.ndarray) -> np.ndarray:
    """
    Calculates the admissible velocity range (dynamic window) based on:
    1. Hardware limits (V_hw)
    2. Acceleration/deceleration limits (V_acc)
    
    x = [x, y, yaw, v, omega]
    Returns: [v_min, v_max, w_min, w_max]
    """
    # 1. Hardware/Config Limits
    v_min_hw = config.MIN_VELOCITY
    v_max_hw = config.MAX_VELOCITY
    w_min_hw = -config.MAX_YAWRATE
    w_max_hw = config.MAX_YAWRATE

    # 2. Acceleration Limits (using current velocity x[3] and x[4])
    v_min_acc = x[3] - config.MAX_DECELERATION * config.SIM_PERIOD
    v_max_acc = x[3] + config.MAX_ACCELERATION * config.SIM_PERIOD
    w_min_acc = x[4] - config.MAX_D_YAWRATE * config.SIM_PERIOD
    w_max_acc = x[4] + config.MAX_D_YAWRATE * config.SIM_PERIOD

    # Combine limits to find the Dynamic Window (Dw)
    v_min = max(v_min_hw, v_min_acc)
    v_max = min(v_max_hw, v_max_acc)
    w_min = max(w_min_hw, w_min_acc)
    w_max = min(w_max_hw, w_max_acc)

    # Sanity checks
    if v_min > v_max: v_min = v_max = x[3]
    if w_min > w_max: w_min = w_max = x[4]

    return np.array([v_min, v_max, w_min, w_max])


def predict_trajectory(x_init: np.ndarray, v: float, omega: float) -> np.ndarray:
    """
    Simulates the robot's state over the prediction time (PREDICT_TIME) 
    using control input (v, omega) and Euler integration.
    """
    time = 0.0
    # Copy state to ensure initial state is not modified during simulation
    state = np.array(x_init, dtype=float)
    traj = [state.copy()]

    while time <= config.PREDICT_TIME:
        # Kinematic model update (x, y, yaw)
        state[0] += state[3] * math.cos(state[2]) * config.SIM_PERIOD # dx
        state[1] += state[3] * math.sin(state[2]) * config.SIM_PERIOD # dy
        state[2] += state[4] * config.SIM_PERIOD                      # dyaw
        state[2] = normalize_angle(state[2])

        # Apply control input (v, omega) for the next step
        state[3] = v
        state[4] = omega

        traj.append(state.copy())
        time += config.SIM_PERIOD

    return np.vstack(traj)


# ============================================================
# COSTS
# ============================================================

def cost_obstacle(trajectory: np.ndarray, obstacles_xy: np.ndarray) -> float:
    """
    Calculates obstacle cost. Returns infinity if collision is detected.
    Cost is inversely proportional to clearance distance (1/clearance).
    """
    if obstacles_xy.size == 0:
        return 0.0

    # Safety radius: physical size + safety padding
    safety_margin = config.ROBOT_RADIUS + config.FOOTPRINT_PADDING
    min_dist = float("inf")

    for pt in trajectory[:, 0:2]:
        # Calculate distance from trajectory point to all obstacle points
        dists = np.linalg.norm(pt - obstacles_xy, axis=1)
        cur_min = np.min(dists)
        min_dist = min(min_dist, cur_min)

        # Check for immediate collision
        if cur_min <= safety_margin:
            return float("inf") # Collision detected

    # Only penalize obstacles within the observation range
    if min_dist < config.OBS_RANGE:
        clearance = max(0.001, min_dist - safety_margin)
        # Cost is 1 / clearance. High cost when clearance is near zero.
        return 1.0 / clearance

    return 0.0


def cost_to_goal(trajectory: np.ndarray, goal: np.ndarray) -> float:
    """
    Calculates the heading cost. Penalizes trajectories that result in a 
    large final angular error relative to the goal position.
    """
    final_state = trajectory[-1, :]
    dx = goal[0] - final_state[0]
    dy = goal[1] - final_state[1]
    
    target_yaw = math.atan2(dy, dx)
    yaw_error = normalize_angle(target_yaw - final_state[2])
    
    # Cost is the absolute magnitude of the final yaw error
    return abs(yaw_error)


def cost_speed(v: float) -> float:
    """
    Calculates the velocity cost. Rewards high linear velocity.
    Cost is 0 at MAX_VELOCITY, and 1 at 0 m/s.
    """
    return 1.0 - (v / config.MAX_VELOCITY)


def dwa_control(x: np.ndarray, goal: np.ndarray, obstacles_xy: np.ndarray):
    """
    Core DWA loop: searches for the best velocity command (v, w) 
    by minimizing the total cost function over all sampled trajectories.
    """
    dw = calculate_dynamic_window(x)

    best_u = np.array([0.0, 0.0])
    best_traj = np.array([x])
    min_cost = float("inf")

    # Sample velocities within the dynamic window
    v_samples = np.linspace(dw[0], dw[1], config.VELOCITY_SAMPLES)
    w_samples = np.linspace(dw[2], dw[3], config.YAWRATE_SAMPLES)

    sample_count = 0

    if v_samples.size == 0 or w_samples.size == 0:
        return best_u, best_traj, sample_count

    for v in v_samples:
        for w in w_samples:
            traj = predict_trajectory(x, v, w)

            # Calculate individual costs
            g_cost = cost_to_goal(traj, goal)
            o_cost = cost_obstacle(traj, obstacles_xy)
            s_cost = cost_speed(v)
            
            # Skip collision paths
            if o_cost == float("inf"):
                continue

            # Calculate total weighted cost
            total = (
                config.TO_GOAL_COST_GAIN * g_cost
                + config.OBSTACLE_COST_GAIN * o_cost
                + config.SPEED_COST_GAIN * s_cost
            )

            sample_count += 1

            # Update best trajectory
            if total < min_cost:
                min_cost = total
                best_u = np.array([v, w])
                best_traj = traj

    # If all sampled trajectories result in collision, stop immediately.
    if min_cost == float("inf"):
        return np.array([0.0, 0.0]), np.array([x]), sample_count

    # --- Deadlock Escape (Stuck Prevention) ---
    # If DWA suggests stopping and the robot is already near-stopped, 
    # force a rotation to break the deadlock.
    if (
        abs(best_u[0]) < config.ROBOT_STUCK_FLAG_CONS
        and abs(x[3]) < config.ROBOT_STUCK_FLAG_CONS
    ):
        # Force maximum negative yawrate (spin in place)
        best_u[1] = -config.MAX_YAWRATE
    # ------------------------------------------

    return best_u, best_traj, sample_count


# ============================================================
# ROS2 NODE CLASS
# ============================================================

class DWATurtleBotPlanner(Node):
    def __init__(self):
        super().__init__("dwa_local_planner")
        self.get_logger().info("DWA Local Planner online, awaiting target...")

        # --- QoS Profiles ---
        # Best effort for sensor data (LaserScan)
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        # Reliable for state data (Odometry)
        qos_odom = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_default = QoSProfile(depth=10)

        # state = [x, y, yaw, v, omega]
        self.state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

        # Goal is now pre-set to the coordinates from RViz log: (1.76, 0.47)
        self.goal: np.ndarray | None = np.array([1.76203, 0.469735])
        self.get_logger().info(f"Default goal set: ({self.goal[0]:.2f}, {self.goal[1]:.2f}). Planner is active.")


        # Obstacle data from LaserScan (x, y coordinates in world/odom frame)
        self.obstacles_xy = np.empty((0, 2))

        # --- Publishers ---
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", qos_default)
        self.marker_pub = self.create_publisher(MarkerArray, "dwa_traj_array", qos_default)

        # --- Subscribers ---
        self.create_subscription(Odometry, "odom", self.odom_callback, qos_odom)
        self.create_subscription(LaserScan, "scan", self.laser_callback, qos_sensor)
        # Goal subscriber is still active to allow setting new goals from RViz
        self.create_subscription(PoseStamped, "/goal_pose", self.goal_callback, qos_default)

        # --- Timer (Main Loop) ---
        self.timer = self.create_timer(config.SIM_PERIOD, self.timer_callback)

    # --------------------------------------------------------
    # CALLBACKS
    # --------------------------------------------------------

    def goal_callback(self, msg: PoseStamped):
        """Updates the goal position from RViz's 2D Goal Pose publisher."""
        self.goal = np.array([msg.pose.position.x, msg.pose.position.y])
        self.get_logger().info(f"Goal received from RViz: ({self.goal[0]:.2f}, {self.goal[1]:.2f}) in {msg.header.frame_id}")

    def odom_callback(self, msg: Odometry):
        """Updates the current robot state from Odometry data."""
        self.state[0] = msg.pose.pose.position.x
        self.state[1] = msg.pose.pose.position.y
        self.state[2] = get_yaw_from_quaternion(msg.pose.pose.orientation)
        self.state[3] = msg.twist.twist.linear.x
        self.state[4] = msg.twist.twist.angular.z

    def laser_callback(self, msg: LaserScan):
        """Processes LaserScan data into (x, y) obstacle coordinates in the world frame."""
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # Filter points by max/min range and OBS_RANGE limit
        valid = (
            (ranges < msg.range_max)
            & (ranges > msg.range_min)
            & (ranges <= config.OBS_RANGE)
        )

        ranges = ranges[valid]
        angles = angles[valid]

        if ranges.size == 0:
            self.obstacles_xy = np.empty((0, 2))
            return

        # 1. Convert to local Cartesian (robot frame)
        x_local = ranges * np.cos(angles)
        y_local = ranges * np.sin(angles)

        # 2. Rotate to world frame
        yaw = self.state[2]
        R = np.array(
            [
                [math.cos(yaw), -math.sin(yaw)],
                [math.sin(yaw), math.cos(yaw)],
            ]
        )
        local = np.vstack((x_local, y_local))
        world_rotated = (R @ local).T

        # 3. Translate to world frame (odom)
        self.obstacles_xy = world_rotated + self.state[0:2]

    # --------------------------------------------------------
    # MARKER VISUALIZATION
    # --------------------------------------------------------

    def publish_trajectory_marker(self, trajectory: np.ndarray):
        """Publishes the best trajectory as a green LINE_STRIP marker."""
        marker_array = MarkerArray()

        line = Marker()
        line.header.frame_id = "odom"
        line.header.stamp = self.get_clock().now().to_msg()
        line.ns = "dwa_path"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD

        line.scale.x = 0.05
        line.color.a = 0.8
        line.color.g = 1.0 # Green line

        # Add all predicted points to the line strip
        for st in trajectory:
            p = PoseStamped().pose.position
            p.x = float(st[0])
            p.y = float(st[1])
            line.points.append(p)

        marker_array.markers.append(line)
        self.marker_pub.publish(marker_array)

    def publish_footprint_marker(self, x: np.ndarray):
        """Publishes the rectangular robot footprint as a red LINE_LIST marker."""
        marker_array = MarkerArray()

        footprint_marker = Marker()
        footprint_marker.header.frame_id = "odom"
        footprint_marker.header.stamp = self.get_clock().now().to_msg()
        footprint_marker.ns = "robot_footprint"
        footprint_marker.id = 1
        footprint_marker.type = Marker.LINE_LIST
        footprint_marker.action = Marker.ADD

        footprint_marker.scale.x = 0.02 # Line thickness
        footprint_marker.color.a = 1.0
        footprint_marker.color.r = 1.0 # Red footprint

        # Define corners relative to the robot center
        L = config.ROBOT_LENGTH / 2.0
        W = config.ROBOT_WIDTH / 2.0
        local_corners = np.array([[L, W], [L, -W], [-L, -W], [-L, W]])
        edges = [(0, 1), (1, 2), (2, 3), (3, 0)] # Edges to connect

        # Transformation matrix (rotation + translation)
        yaw = x[2]
        R = np.array(
            [
                [math.cos(yaw), -math.sin(yaw)],
                [math.sin(yaw), math.cos(yaw)],
            ]
        )
        # Apply rotation and translation
        world_corners = (R @ local_corners.T).T + x[0:2]

        # Add points (P1, P2), (P2, P3), (P3, P4), (P4, P1) to the LINE_LIST
        for i, j in edges:
            p1 = PoseStamped().pose.position
            p1.x, p1.y = world_corners[i]
            footprint_marker.points.append(p1)

            p2 = PoseStamped().pose.position
            p2.x, p2.y = world_corners[j]
            footprint_marker.points.append(p2)

        marker_array.markers.append(footprint_marker)
        self.marker_pub.publish(marker_array)

    # --------------------------------------------------------
    # MAIN LOOP
    # --------------------------------------------------------

    def timer_callback(self):
        cmd = Twist()
        
        # 1. Check for goal existence (always true now unless cleared externally)
        if self.goal is None:
            self.cmd_vel_pub.publish(cmd)
            self.get_logger().info("Goal was cleared. Waiting for RViz goal on /goal_pose ...")
            return

        # 2. Check for goal reached
        dist = np.linalg.norm(self.state[0:2] - self.goal)
        if dist < config.GOAL_THRESHOLD:
            self.get_logger().info("Goal reached. Holding position.")
            self.cmd_vel_pub.publish(cmd)
            # Stop the markers for visualization
            self.publish_trajectory_marker(np.array([self.state]))
            return

        # 3. Execute DWA
        u, best_traj, samples = dwa_control(self.state, self.goal, self.obstacles_xy)

        # 4. Publish commands
        cmd.linear.x = float(u[0])
        cmd.angular.z = float(u[1])
        self.cmd_vel_pub.publish(cmd)

        # 5. Publish visualizations
        self.publish_trajectory_marker(best_traj)
        self.publish_footprint_marker(self.state)

        # 6. Logging
        self.get_logger().info(
            f"DWA | d={dist:.2f} m | obs={len(self.obstacles_xy)} | samp={samples} | "
            f"v={u[0]:.2f} m/s | w={u[1]:.2f} rad/s"
        )


# ============================================================
# MAIN
# ============================================================

def main(args=None):
    rclpy.init(args=args)
    node = DWATurtleBotPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Planner error: {e}")
    finally:
        # Emergency stop on exit
        stop = Twist()
        if hasattr(node, 'cmd_vel_pub'):
            node.cmd_vel_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
