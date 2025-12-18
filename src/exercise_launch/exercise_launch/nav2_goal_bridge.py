#!/usr/bin/env python3
"""
Nav2 Goal Bridge Node

Bridges RViz2's "2D Goal Pose" tool (which publishes to /goal_pose) 
to Nav2's NavigateToPose action server (/navigate_to_pose).

This allows clicking on the map in RViz2 to set navigation goals.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class Nav2GoalBridge(Node):
    def __init__(self):
        super().__init__('nav2_goal_bridge')
        
        # Action client for Nav2's NavigateToPose action
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Subscribe to goal_pose topic from RViz2's "2D Goal Pose" tool
        self._goal_subscription = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_callback,
            10
        )
        
        self.get_logger().info('Nav2 Goal Bridge started. Waiting for NavigateToPose action server...')
        self.get_logger().info('Click on the map in RViz2 using the "2D Goal Pose" tool to set navigation goals.')
    
    def goal_callback(self, msg: PoseStamped):
        """Callback when a goal pose is received from RViz2"""
        self.get_logger().info(f'Received goal pose: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}')
        
        # Wait for action server to be available
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose action server not available!')
            return
        
        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg
        
        # Send goal to action server
        self.get_logger().info('Sending goal to Nav2...')
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Callback when goal is accepted/rejected"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2!')
            return
        
        self.get_logger().info('Goal accepted by Nav2! Robot is navigating...')
        
        # Get result when goal completes
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future):
        """Callback when goal execution completes"""
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info('✓ Navigation goal succeeded!')
        else:
            self.get_logger().warn(f'Navigation goal completed with status: {status}')


def main(args=None):
    rclpy.init(args=args)
    
    node = Nav2GoalBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
