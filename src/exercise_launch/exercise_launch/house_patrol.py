#!/usr/bin/env python3
"""
House patrol script (Phase 5).

Loads 3 named waypoints (kitchen, bedroom, home) from:
  /home/dio/ros2_ws/config/patrol_waypoints.yaml

Then navigates to them sequentially using Nav2 Simple Commander.
If any goal FAILS, it logs and continues to the next waypoint.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Any, List, Optional

import rclpy

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

import yaml


DEFAULT_WAYPOINTS_PATH = "/home/dio/ros2_ws/config/patrol_waypoints.yaml"


@dataclass(frozen=True)
class Waypoint:
    name: str
    frame_id: str
    x: float
    y: float
    yaw: float = 0.0

    def to_pose_stamped(self, navigator: BasicNavigator) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = navigator.get_clock().now().to_msg()
        msg.pose.position.x = float(self.x)
        msg.pose.position.y = float(self.y)
        msg.pose.position.z = 0.0

        # yaw -> quaternion
        cy = math.cos(self.yaw * 0.5)
        sy = math.sin(self.yaw * 0.5)
        msg.pose.orientation.w = cy
        msg.pose.orientation.z = sy
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        return msg


def _load_waypoints(path: str) -> List[Waypoint]:
    with open(path, "r", encoding="utf-8") as f:
        data: Dict[str, Any] = yaml.safe_load(f) or {}

    wp = (data.get("waypoints") or {})
    missing = [k for k in ("kitchen", "bedroom", "home") if k not in wp]
    if missing:
        raise ValueError(f"Missing waypoints in YAML: {missing}. Expected keys: kitchen, bedroom, home")

    def parse(name: str) -> Waypoint:
        d = wp[name] or {}
        return Waypoint(
            name=name,
            frame_id=str(d.get("frame_id") or "map"),
            x=float(d["x"]),
            y=float(d["y"]),
            yaw=float(d.get("yaw") or 0.0),
        )

    return [parse("kitchen"), parse("bedroom"), parse("home")]


def _log_result(navigator: BasicNavigator, waypoint: Waypoint, result: TaskResult) -> None:
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info(f"[{waypoint.name}] SUCCEEDED")
    elif result == TaskResult.FAILED:
        navigator.get_logger().error(f"[{waypoint.name}] FAILED (skipping to next waypoint)")
    elif result == TaskResult.CANCELED:
        navigator.get_logger().warn(f"[{waypoint.name}] CANCELED (skipping to next waypoint)")
    else:
        navigator.get_logger().warn(f"[{waypoint.name}] Finished with unknown result: {result}")


def main(args=None) -> None:
    rclpy.init(args=args)
    navigator = BasicNavigator()

    # Load waypoints from YAML
    try:
        waypoints = _load_waypoints(DEFAULT_WAYPOINTS_PATH)
    except Exception as e:
        navigator.get_logger().error(
            f"Failed to load waypoints from {DEFAULT_WAYPOINTS_PATH}: {e!r}. "
            f"Run: ros2 run exercise_launch patrol_waypoint_recorder"
        )
        rclpy.shutdown()
        return

    navigator.get_logger().info("Waiting for Nav2 to become active...")
    navigator.waitUntilNav2Active()

    # Set initial pose at (0,0,0) in map
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = "map"
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    for wp in waypoints:
        goal = wp.to_pose_stamped(navigator)
        navigator.get_logger().info(f"Navigating to [{wp.name}] x={wp.x:.2f}, y={wp.y:.2f} (frame={wp.frame_id})")
        navigator.goToPose(goal)

        # Wait until completion, but keep it robust and non-blocking.
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback is not None and feedback.distance_remaining is not None:
                navigator.get_logger().info(f"[{wp.name}] distance_remaining={feedback.distance_remaining:.2f}m")
            rclpy.spin_once(navigator, timeout_sec=0.5)

        result = navigator.getResult()
        _log_result(navigator, wp, result)

    navigator.get_logger().info("Patrol cycle complete. Exiting.")
    rclpy.shutdown()

