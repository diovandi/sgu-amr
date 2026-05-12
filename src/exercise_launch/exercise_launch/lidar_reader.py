import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


class LidarReaderNode(Node):
    """
    Phase 2: Sensor Processing Node
    
    Reads LIDAR data from /scan topic with Best Effort QoS to match Gazebo's
    sensor publisher. Processes LaserScan messages and logs key statistics.
    """
    
    def __init__(self):
        super().__init__('lidar_reader_node')
        
        # Parameters for logging behavior
        self.declare_parameter('log_frequency', 10)  # Log every N scans
        self.declare_parameter('verbose', False)  # Print detailed stats
        
        self.log_frequency = self.get_parameter('log_frequency').get_parameter_value().integer_value
        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value
        
        # Counter for scan messages
        self.scan_count = 0
        
        # QoS profile matching Gazebo's Best Effort sensor output
        # This is CRITICAL: Gazebo sensors publish with Best Effort QoS by default
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        
        # Subscriber for the laser scan with sensor QoS
        self.subscription_ = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            sensor_qos,
        )
        
        self.get_logger().info('LIDAR Reader node started')
        self.get_logger().info(f'Subscribed to /scan with Best Effort QoS')
        self.get_logger().info(f'Log frequency: every {self.log_frequency} scans')
    
    def scan_callback(self, msg: LaserScan):
        """
        Process incoming LaserScan messages and extract statistics.
        
        Args:
            msg: LaserScan message from /scan topic
        """
        self.scan_count += 1
        
        # Extract basic scan parameters
        num_ranges = len(msg.ranges)
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        range_min = msg.range_min
        range_max = msg.range_max
        frame_id = msg.header.frame_id
        
        # Process range data
        valid_ranges = []
        invalid_count = 0
        
        for scan_range in msg.ranges:
            if math.isinf(scan_range) or math.isnan(scan_range):
                invalid_count += 1
            elif scan_range >= range_min and scan_range <= range_max:
                valid_ranges.append(scan_range)
            else:
                invalid_count += 1
        
        # Calculate statistics
        if valid_ranges:
            min_range = min(valid_ranges)
            max_range = max(valid_ranges)
            mean_range = sum(valid_ranges) / len(valid_ranges)
            valid_count = len(valid_ranges)
        else:
            min_range = float('inf')
            max_range = 0.0
            mean_range = 0.0
            valid_count = 0
        
        # Log statistics periodically to avoid console spam
        if self.scan_count % self.log_frequency == 0:
            self.get_logger().info(
                f'=== LIDAR Scan #{self.scan_count} ==='
            )
            self.get_logger().info(
                f'Frame: {frame_id} | '
                f'Total readings: {num_ranges} | '
                f'Valid: {valid_count} | '
                f'Invalid: {invalid_count}'
            )
            self.get_logger().info(
                f'Angle range: [{math.degrees(angle_min):.1f}°, '
                f'{math.degrees(angle_max):.1f}°] | '
                f'Increment: {math.degrees(angle_increment):.3f}°'
            )
            self.get_logger().info(
                f'Range limits: [{range_min:.2f}m, {range_max:.2f}m]'
            )
            
            if valid_count > 0:
                self.get_logger().info(
                    f'Valid range stats: '
                    f'min={min_range:.2f}m, '
                    f'max={max_range:.2f}m, '
                    f'mean={mean_range:.2f}m'
                )
            else:
                self.get_logger().warn('No valid range readings in this scan!')
            
            # Verbose mode: print first scan details
            if self.verbose and self.scan_count == self.log_frequency:
                self.get_logger().info('--- Verbose Details ---')
                self.get_logger().info(
                    f'Scan time: {msg.scan_time:.4f}s | '
                    f'Time increment: {msg.time_increment:.6f}s'
                )
                self.get_logger().info(
                    f'Intensities available: {len(msg.intensities) > 0}'
                )
        
        # Log first scan with full details regardless of frequency
        if self.scan_count == 1:
            self.get_logger().info('=== First LIDAR Scan Received ===')
            self.get_logger().info(
                f'Frame: {frame_id} | '
                f'Total readings: {num_ranges} | '
                f'Angle range: [{math.degrees(angle_min):.1f}°, '
                f'{math.degrees(angle_max):.1f}°] | '
                f'Increment: {math.degrees(angle_increment):.3f}°'
            )
            self.get_logger().info(
                f'Range limits: [{range_min:.2f}m, {range_max:.2f}m]'
            )
            # Debug: Check first few range values
            sample_size = min(10, num_ranges)
            sample_ranges = msg.ranges[:sample_size]
            self.get_logger().info(
                f'First {sample_size} range values: {sample_ranges}'
            )
            if valid_count > 0:
                self.get_logger().info(
                    f'Valid readings: {valid_count}/{num_ranges} | '
                    f'Min range: {min_range:.2f}m | '
                    f'Max range: {max_range:.2f}m'
                )
            else:
                self.get_logger().warn(
                    f'No valid readings! All {num_ranges} readings are invalid. '
                    f'Check if robot is in empty space or LIDAR configuration.'
                )


def main(args=None):
    rclpy.init(args=args)
    node = LidarReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.get_logger().info('Shutting down LIDAR Reader node...')
            node.destroy_node()
        except Exception:
            pass  # Node may already be destroyed
        try:
            rclpy.shutdown()
        except Exception:
            pass  # Shutdown may have already been called


if __name__ == '__main__':
    main()
