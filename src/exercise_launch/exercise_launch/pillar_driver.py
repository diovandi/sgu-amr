import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from irobot_create_msgs.msg import HazardDetectionVector


class PillarDriverNode(Node):
    def __init__(self):
        super().__init__('pillar_driver_node')

        # Core controller parameters
        self.declare_parameter('linear_gain', 0.3)
        self.declare_parameter('angular_gain', 0.3)
        self.declare_parameter('stop_distance', 0.5)
        # Small extra margin so we declare success slightly before the exact
        # stop distance, which avoids oscillating between \"target\" and
        # \"no pillar\" when the cluster geometry starts to break down.
        self.declare_parameter('stop_margin', 0.05)
        self.declare_parameter('search_angle', 1.0)  # +/- radians from front
        self.declare_parameter('min_detection_range', 0.3)  # ignore self-hit

        # LiDAR angle offset: correction for LiDAR frame rotation relative to base_link.
        # Positive offset rotates LiDAR frame counter-clockwise.
        self.declare_parameter('lidar_angle_offset', 0.0)

        # Obstacle avoidance (for dock / obstacles directly ahead)
        self.declare_parameter('obstacle_distance', 0.7)      # m
        self.declare_parameter('obstacle_cone_angle', 0.5)    # rad, ~±30°
        self.declare_parameter('obstacle_turn_speed', 0.7)    # rad/s

        # Pillar shape + clustering / scoring parameters
        # Approximate physical dimensions of the pillar in the world:
        # model.sdf uses radius=0.3, length=1.0 -> diameter ≈ 0.6m.
        self.declare_parameter('pillar_diameter', 0.6)            # m
        self.declare_parameter('pillar_width_tolerance', 0.3)     # m
        # Reasonable distance band in which we expect to see the pillar.
        self.declare_parameter('pillar_min_range', 0.5)           # m
        self.declare_parameter('pillar_max_range', 4.0)           # m
        # How aggressively to break clusters when the range jumps.
        self.declare_parameter('cluster_break_distance', 0.25)    # m
        # Weights for the cluster cost function:
        #   J = w_angle * |angle| + w_dist * range + w_width * |width - pillar_diameter|
        self.declare_parameter('pillar_angle_weight', 1.0)
        self.declare_parameter('pillar_distance_weight', 0.5)
        self.declare_parameter('pillar_width_weight', 0.5)

        # Bumper / hazard recovery using Create3 hazard detections
        # NOTE: we keep these parameters for backwards-compatibility, but the
        # new recovery uses an explicit multi-stage sequence instead of a
        # simple timed spin.
        self.declare_parameter('hazard_recovery_duration', 5.0)   # seconds (unused)
        self.declare_parameter('hazard_recovery_turn_speed', 1.0) # rad/s (unused)
        # LiDAR-based "crash" trigger: if something is extremely close in
        # front of the robot, we treat it like a bumper hit even if the
        # simulated hazards don't fire.
        self.declare_parameter('crash_distance', 0.2)  # m

        self.linear_gain = self.get_parameter('linear_gain').get_parameter_value().double_value
        self.angular_gain = self.get_parameter('angular_gain').get_parameter_value().double_value
        self.stop_distance = self.get_parameter('stop_distance').get_parameter_value().double_value
        self.stop_margin = self.get_parameter('stop_margin').get_parameter_value().double_value
        self.search_angle = self.get_parameter('search_angle').get_parameter_value().double_value
        self.min_detection_range = self.get_parameter('min_detection_range').get_parameter_value().double_value
        self.lidar_angle_offset = self.get_parameter('lidar_angle_offset').get_parameter_value().double_value
        self.obstacle_distance = self.get_parameter('obstacle_distance').get_parameter_value().double_value
        self.obstacle_cone_angle = self.get_parameter('obstacle_cone_angle').get_parameter_value().double_value
        self.obstacle_turn_speed = self.get_parameter('obstacle_turn_speed').get_parameter_value().double_value
        # Pillar geometry + clustering / scoring config
        self.pillar_diameter = self.get_parameter('pillar_diameter').get_parameter_value().double_value
        self.pillar_width_tolerance = self.get_parameter('pillar_width_tolerance').get_parameter_value().double_value
        self.pillar_min_range = self.get_parameter('pillar_min_range').get_parameter_value().double_value
        self.pillar_max_range = self.get_parameter('pillar_max_range').get_parameter_value().double_value
        self.cluster_break_distance = self.get_parameter('cluster_break_distance').get_parameter_value().double_value
        self.pillar_angle_weight = self.get_parameter('pillar_angle_weight').get_parameter_value().double_value
        self.pillar_distance_weight = self.get_parameter('pillar_distance_weight').get_parameter_value().double_value
        self.pillar_width_weight = self.get_parameter('pillar_width_weight').get_parameter_value().double_value
        # Keep reading these in case they are used elsewhere / logged
        self.hazard_recovery_duration = self.get_parameter('hazard_recovery_duration').get_parameter_value().double_value
        self.hazard_recovery_turn_speed = self.get_parameter('hazard_recovery_turn_speed').get_parameter_value().double_value
        self.crash_distance = self.get_parameter('crash_distance').get_parameter_value().double_value

        # Startup sequencing so the robot fully initializes before we move:
        #  - initial_delay_duration: time to wait doing nothing
        #  - initial_backoff_duration: time to drive backwards off the dock
        self.initial_delay_duration = 5.0    # seconds
        self.initial_backoff_duration = 2.0  # seconds
        # After the explicit startup backoff, give the internal motion_control
        # node a short grace period to finish any autonomous dock behavior
        # before we start treating odom-based backwards motion as a "crash".
        self.external_backoff_grace = 1.0    # seconds
        self.start_time = self.get_clock().now()

        # Multi-stage hazard recovery state machine
        # States: None, 'pause', 'back', 'turn', 'forward_clear'
        self.recovery_state = None
        self.recovery_state_start = None

        # Track what the robot is actually doing (from odometry) versus what
        # we *think* we commanded. This lets us detect when the internal
        # motion_control node is running an autonomous safety/docking behavior
        # (e.g. backing off the dock) and overriding our /cmd_vel.
        self.current_linear_x = 0.0
        self.last_command_v = 0.0

        # Publisher for velocity commands (TwistStamped for TurtleBot4/Create3)
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # Publisher for RViz visualization marker
        self.marker_publisher_ = self.create_publisher(Marker, '/pillar_marker', 10)

        # QoS profile for sensor data (Best Effort to match Gazebo's LiDAR publisher)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subscriber for the laser scan with sensor QoS
        self.subscription_ = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            sensor_qos,
        )

        # Subscriber for Create3 hazard detections (front bumper, cliffs, etc.)
        self.hazard_sub_ = self.create_subscription(
            HazardDetectionVector,
            '/hazards_vector',
            self.hazard_callback,
            10,
        )

        # Odometry subscriber so we can compare desired vs actual motion and
        # detect when an external autonomous behavior (like dock backoff) is
        # running.
        self.odom_sub_ = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
        )

        # Counter for one-shot LiDAR diagnostics
        self.scan_count = 0

        # Slow-down counter for pillar cluster debug logs
        self.cluster_debug_decimate = 30

        # Once we have successfully reached the pillar, latch a simple
        # \"mission complete\" flag so we don't immediately start searching
        # for a new pillar again.
        self.mission_complete = False

        self.get_logger().info('Pillar driver started')
        self.get_logger().info(f'lidar_angle_offset = {math.degrees(self.lidar_angle_offset):.1f} deg')
        self.get_logger().info(
            'Pillar params: '
            f'diameter={self.pillar_diameter:.2f}m, '
            f'width_tol={self.pillar_width_tolerance:.2f}m, '
            f'range=[{self.pillar_min_range:.2f},{self.pillar_max_range:.2f}]m'
        )

    def odom_callback(self, msg: Odometry):
        """Store the current forward velocity of the robot from odometry."""
        self.current_linear_x = msg.twist.twist.linear.x

    def hazard_callback(self, msg: HazardDetectionVector):
        """Start a multi-stage recovery sequence when a front hazard is seen."""
        if not msg.detections:
            return

        # For this exercise we treat any hazard as a "front crash".
        # Start at the PAUSE stage; listener_callback will run the sequence:
        #   pause 2s -> back 2s -> turn 45deg left -> drive forward until clear.
        self.recovery_state = 'pause'
        self.recovery_state_start = self.get_clock().now()
        self.get_logger().info(f'Hazard detected ({len(msg.detections)} events). Starting recovery sequence.')

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi] range where 0 = forward."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def listener_callback(self, msg):
        cmd_vel_msg = TwistStamped()
        cmd_vel_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_vel_msg.header.frame_id = 'base_link'

        now = self.get_clock().now()

        # PHASE 0: Explicit hazard recovery state machine. This overrides all
        # other behavior until the robot has backed up, turned, and driven
        # forward past the obstacle.
        if self.recovery_state is not None:
            dt = (now - self.recovery_state_start).nanoseconds / 1e9

            if self.recovery_state == 'pause':
                # 0–2s: full stop
                if dt < 2.0:
                    cmd_vel_msg.twist.linear.x = 0.0
                    cmd_vel_msg.twist.angular.z = 0.0
                    self.publisher_.publish(cmd_vel_msg)
                    self.get_logger().info('Recovery: pause after crash (2.0s total).')
                    return
                else:
                    # Move to backing up phase
                    self.recovery_state = 'back'
                    self.recovery_state_start = now
                    dt = 0.0

            if self.recovery_state == 'back':
                # 2–4s: back straight for 2s
                if dt < 2.0:
                    cmd_vel_msg.twist.linear.x = -0.2
                    cmd_vel_msg.twist.angular.z = 0.0
                    self.publisher_.publish(cmd_vel_msg)
                    self.get_logger().info('Recovery: backing up for 2.0s.')
                    return
                else:
                    # Move to turn phase
                    self.recovery_state = 'turn'
                    self.recovery_state_start = now
                    dt = 0.0

            if self.recovery_state == 'turn':
                # Turn 45 degrees left in place.
                turn_speed = 0.7  # rad/s
                target_angle = math.pi / 4.0  # 45 degrees
                turn_duration = target_angle / turn_speed

                if dt < turn_duration:
                    cmd_vel_msg.twist.linear.x = 0.0
                    cmd_vel_msg.twist.angular.z = turn_speed
                    self.publisher_.publish(cmd_vel_msg)
                    self.get_logger().info('Recovery: turning 45deg left.')
                    return
                else:
                    # Move to forward-clear phase
                    self.recovery_state = 'forward_clear'
                    self.recovery_state_start = now
                    # Do not return here; we want to use LiDAR below to decide
                    # when we are past the obstacle.

        self.scan_count += 1
        
        # Log LiDAR parameters once for debugging
        if self.scan_count == 1:
            self.get_logger().info(
                'LiDAR: '
                f'angle_min={math.degrees(msg.angle_min):.1f}deg, '
                f'angle_max={math.degrees(msg.angle_max):.1f}deg, '
                f'num_ranges={len(msg.ranges)}, frame={msg.header.frame_id}',
            )

        # PHASE 1: Global startup pause + backoff to let the robot fully
        # initialize its controllers and sensors before we start navigating.
        startup_elapsed = (now - self.start_time).nanoseconds / 1e9
        if startup_elapsed < self.initial_delay_duration:
            # Just wait; don't send any motion during this time.
            cmd_vel_msg.twist.linear.x = 0.0
            cmd_vel_msg.twist.angular.z = 0.0
            self.publisher_.publish(cmd_vel_msg)
            self.get_logger().info(
                f'Initial startup wait ({startup_elapsed:.1f}/{self.initial_delay_duration:.1f}s)'
            )
            return
        elif startup_elapsed < self.initial_delay_duration + self.initial_backoff_duration:
            # After waiting, drive straight backwards off the dock.
            backoff_elapsed = startup_elapsed - self.initial_delay_duration
            cmd_vel_msg.twist.linear.x = -0.2
            cmd_vel_msg.twist.angular.z = 0.0
            self.publisher_.publish(cmd_vel_msg)
            self.get_logger().info(
                f'Initial backoff phase ({backoff_elapsed:.1f}/{self.initial_backoff_duration:.1f}s)'
            )
            return

        # PHASE 2: Detect when motion_control is running an internal autonomous
        # safety/docking behavior. If we are commanding forward but the robot
        # is actually moving backwards, treat that as an external "crash"
        # reaction and start our own recovery sequence so that, once control
        # returns to us, we steer around the obstacle instead of driving
        # straight back into it.
        startup_elapsed = (now - self.start_time).nanoseconds / 1e9
        external_backoff = (
            self.recovery_state is None
            and self.last_command_v > 0.05
            and self.current_linear_x < -0.05
            and startup_elapsed > (
                self.initial_delay_duration
                + self.initial_backoff_duration
                + self.external_backoff_grace
            )
        )
        if external_backoff:
            self.get_logger().warn(
                'Detected external backoff (autonomous safety/dock behavior); '
                'starting recovery sequence.'
            )
            self.recovery_state = 'pause'
            self.recovery_state_start = now
            cmd_vel_msg.twist.linear.x = 0.0
            cmd_vel_msg.twist.angular.z = 0.0
            self.publisher_.publish(cmd_vel_msg)
            return

        # If we've already completed the mission (successfully reached the
        # pillar once), hold position and do not look for new targets.
        if self.mission_complete:
            cmd_vel_msg.twist.linear.x = 0.0
            cmd_vel_msg.twist.angular.z = 0.0
            self.publisher_.publish(cmd_vel_msg)
            return

        # Find candidate pillar returns using a cluster-based detector in the
        # forward-facing cone, while also tracking the closest obstacle
        # directly in front of the robot (for dock / obstacle avoidance).
        # NOTE: obstacle tracking remains point-wise so previous crash /
        # avoidance behaviour is preserved; only the pillar-selection logic
        # changes.

        # For obstacle avoidance (front cone using corrected angle)
        min_front_dist = msg.range_max

        clusters = []
        current_cluster = None
        last_range = None

        for i, scan_range in enumerate(msg.ranges):
            # Calculate the RAW angle for this range index (in LiDAR frame)
            raw_angle = msg.angle_min + (i * msg.angle_increment)

            # Apply LiDAR frame offset to convert to robot body frame
            # Then normalize to [-pi, pi] where 0 = robot forward
            angle = self.normalize_angle(raw_angle + self.lidar_angle_offset)

            # Track the closest obstacle in a narrower front cone
            if (abs(angle) < self.obstacle_cone_angle and
                not math.isinf(scan_range) and
                not math.isnan(scan_range)):
                if scan_range < min_front_dist:
                    min_front_dist = scan_range

            # Only consider points within the forward search cone for the pillar
            if abs(angle) > self.search_angle:
                # Break any active cluster when we leave the search cone
                current_cluster = None
                last_range = None
                continue

            # Check if the range is valid and not too close (self-detection)
            valid_for_pillar = (
                not math.isinf(scan_range) and
                not math.isnan(scan_range) and
                scan_range > self.min_detection_range and
                scan_range < msg.range_max
            )
            if not valid_for_pillar:
                # End current cluster on invalid sample
                current_cluster = None
                last_range = None
                continue

            # Either extend the current cluster or start a new one if the
            # range jump is large (likely a separate object edge).
            if (current_cluster is None or
                last_range is None or
                abs(scan_range - last_range) > self.cluster_break_distance):
                current_cluster = {
                    'sum_range': 0.0,
                    'sum_angle': 0.0,
                    'count': 0,
                    'min_angle': angle,
                    'max_angle': angle,
                }
                clusters.append(current_cluster)

            current_cluster['sum_range'] += scan_range
            current_cluster['sum_angle'] += angle
            current_cluster['count'] += 1
            current_cluster['min_angle'] = min(current_cluster['min_angle'], angle)
            current_cluster['max_angle'] = max(current_cluster['max_angle'], angle)

            last_range = scan_range

        # From the clusters inside the search cone, estimate simple geometry
        # (mean range, mean angle, angular span, lateral width) and use a cost
        # function to pick the cluster that best matches the expected pillar.
        min_dist = msg.range_max
        min_dist_angle = 0.0
        min_dist_raw_angle = 0.0  # For debugging / marker
        found_target = False

        best_cost = None
        chosen_debug = None

        for cluster in clusters:
            if cluster['count'] <= 0:
                continue

            mean_range = cluster['sum_range'] / cluster['count']
            mean_angle = cluster['sum_angle'] / cluster['count']
            dtheta = cluster['max_angle'] - cluster['min_angle']
            width = abs(mean_range * dtheta)

            # Basic sanity filters for where we expect the pillar to be.
            if (mean_range < self.min_detection_range or
                mean_range < self.pillar_min_range or
                mean_range > self.pillar_max_range):
                continue

            min_width = max(0.0, self.pillar_diameter - self.pillar_width_tolerance)
            max_width = self.pillar_diameter + self.pillar_width_tolerance
            if width < min_width or width > max_width:
                continue

            # Cost function: prefer clusters that are near the robot's
            # forward axis, at reasonable distance, and with width close to
            # the expected pillar diameter.
            cost = (
                self.pillar_angle_weight * abs(mean_angle) +
                self.pillar_distance_weight * mean_range +
                self.pillar_width_weight * abs(width - self.pillar_diameter)
            )

            if best_cost is None or cost < best_cost:
                best_cost = cost
                min_dist = mean_range
                min_dist_angle = mean_angle
                # Convert back to LiDAR frame for visualization marker
                min_dist_raw_angle = self.normalize_angle(mean_angle - self.lidar_angle_offset)
                found_target = True
                chosen_debug = {
                    'range': mean_range,
                    'angle': mean_angle,
                    'width': width,
                    'cost': cost,
                    'count': cluster['count'],
                }

        # If we are in the final "forward_clear" phase of recovery, drive
        # straight ahead until the obstacle directly in front is past a
        # clearance distance *and* we've driven forward for a minimum time,
        # then resume normal pillar tracking. This ensures we commit to
        # moving past the obstacle before we start turning back toward the
        # pillar.
        if self.recovery_state == 'forward_clear':
            forward_dt = (now - self.recovery_state_start).nanoseconds / 1e9
            min_forward_time = 2.0  # seconds
            clearance_distance = self.obstacle_distance + 0.2  # a bit past obstacle_distance
            if forward_dt >= min_forward_time and min_front_dist > clearance_distance:
                self.get_logger().info('Recovery: obstacle cleared, resuming pillar tracking.')
                self.recovery_state = None
                self.recovery_state_start = None
            else:
                cmd_vel_msg.twist.linear.x = 0.2
                cmd_vel_msg.twist.angular.z = 0.0
                self.publisher_.publish(cmd_vel_msg)
                self.get_logger().info(
                    f'Recovery: driving forward to clear obstacle (front={min_front_dist:.2f}m).'
                )
                return

        # If no explicit hazard message was received but LiDAR says we are
        # essentially on top of something in the front cone, treat it as a
        # crash and start the same recovery sequence. We skip this if we're
        # already very close to the pillar target (within stop distance).
        if (
            self.recovery_state is None
            and min_front_dist < self.crash_distance
            and not (found_target and min_dist < self.stop_distance + 0.1)
        ):
            self.recovery_state = 'pause'
            self.recovery_state_start = now
            self.get_logger().info(
                f'Crash detected by LiDAR (front={min_front_dist:.2f}m). '
                'Starting recovery sequence.'
            )
            # The pause/back/turn/forward_clear logic above will take over on
            # the next callback, so we can fall through and let PHASE 1
            # (initial spin) be skipped because recovery_state is not None.

        # Optional debug for chosen pillar cluster (decimated to avoid spam)
        if chosen_debug is not None and (self.scan_count % self.cluster_debug_decimate == 0):
            self.get_logger().info(
                'Pillar cluster: '
                f"range={chosen_debug['range']:.2f}m, "
                f"angle={math.degrees(chosen_debug['angle']):.1f}deg, "
                f"width={chosen_debug['width']:.2f}m, "
                f"cost={chosen_debug['cost']:.3f}, "
                f"points={chosen_debug['count']}"
            )

        # Decide behavior priority:
        # 1) If no pillar found -> search
        # 2) If pillar is within stop distance -> stop (even if obstacle cone says avoid)
        # 3) Otherwise, avoid obstacles in front
        # 4) Otherwise, drive toward pillar
        if not found_target:
            cmd_vel_msg.twist.linear.x = 0.0
            cmd_vel_msg.twist.angular.z = 0.3  # Slow rotation to search
            self.get_logger().info('No pillar detected in front. Searching...')
        elif min_dist <= (self.stop_distance + self.stop_margin):
            cmd_vel_msg.twist.linear.x = 0.0
            cmd_vel_msg.twist.angular.z = 0.0
            if not self.mission_complete:
                self.get_logger().info(
                    f'Pillar reached! Stopping at {min_dist:.2f}m. Mission complete.'
                )
            self.mission_complete = True
        elif min_front_dist < self.obstacle_distance and not (min_dist < self.stop_distance + 0.1):
            # Only avoid if we are NOT already trying to stop on the pillar.
            cmd_vel_msg.twist.linear.x = 0.0
            cmd_vel_msg.twist.angular.z = self.obstacle_turn_speed
            self.last_command_v = cmd_vel_msg.twist.linear.x
            self.publisher_.publish(cmd_vel_msg)
            self.get_logger().info(
                f'Obstacle at {min_front_dist:.2f}m in front. '
                f'Avoiding base/obstacle (turn left w={self.obstacle_turn_speed:.2f}).'
            )
            return
        else:
            # We found a target, run the P-controller
            angle_error_deg = abs(math.degrees(min_dist_angle))
            
            # Angular velocity - turn towards pillar
            # In TurtleBot4's LiDAR frame, positive angle = left of robot
            # Positive angular.z = turn left (CCW), so the signs match
            angular_vel = self.angular_gain * min_dist_angle
            
            # Clamp angular velocity for safety
            angular_vel = max(-1.0, min(1.0, angular_vel))
            
            # Updated strategy:
            # - If angle > 45 degrees: STOP and turn in place
            # - If angle <= 45 degrees: Drive forward with corrections
            if angle_error_deg > 45.0:
                # NOT ALIGNED ENOUGH: Turn in place first (no forward motion)
                linear_vel = 0.0
                self.get_logger().info(f'Pillar at {min_dist:.2f}m, raw={math.degrees(min_dist_raw_angle):.1f}deg, corrected={math.degrees(min_dist_angle):.1f}deg. TURNING (w={angular_vel:.2f})')
            else:
                # WITHIN 45°: Drive forward toward pillar while turning
                # Slightly slower max speed to reduce chance of bumping the base / dock
                linear_vel = min(self.linear_gain * min_dist, 0.2)
                self.get_logger().info(f'Pillar at {min_dist:.2f}m, raw={math.degrees(min_dist_raw_angle):.1f}deg, corrected={math.degrees(min_dist_angle):.1f}deg. DRIVING (v={linear_vel:.2f}, w={angular_vel:.2f})')
            
            cmd_vel_msg.twist.linear.x = linear_vel
            cmd_vel_msg.twist.angular.z = angular_vel

        # Publish the velocity command
        self.last_command_v = cmd_vel_msg.twist.linear.x
        self.publisher_.publish(cmd_vel_msg)
        
        # Publish the visualization marker (Req. 7)
        # IMPORTANT: use RAW LiDAR angle for visualization in LiDAR frame,
        # otherwise the marker appears rotated relative to the scan points.
        if found_target:
            self.publish_pillar_marker(min_dist, min_dist_raw_angle, msg.header.frame_id)

    def publish_pillar_marker(self, r, theta, frame_id):
        marker = Marker()
        marker.header.frame_id = frame_id  # Publish in the laser's frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "pillar"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        # Convert polar (r, theta) to Cartesian (x, y)
        marker.pose.position.x = r * math.cos(theta)
        marker.pose.position.y = r * math.sin(theta)
        marker.pose.position.z = 0.5  # 0.5m high
        
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.4  # 40cm diameter (matches world)
        marker.scale.y = 0.4
        marker.scale.z = 1.0  # 1m high (matches world)
        
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        marker.lifetime.sec = 1
        
        self.marker_publisher_.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = PillarDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot when the node is shut down
        try:
            stop_msg = TwistStamped()
            stop_msg.header.stamp = node.get_clock().now().to_msg()
            stop_msg.header.frame_id = 'base_link'
            node.publisher_.publish(stop_msg)
            node.get_logger().info('Shutting down Pillar Driver...')
        except Exception:
            pass  # Context may already be invalid
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass  # Already shut down

if __name__ == '__main__':
    main()