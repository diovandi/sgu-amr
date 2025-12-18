#!/usr/bin/env python3
"""
Patrol Waypoint Recorder

One-time helper to capture 3 waypoints from RViz ("Publish Point" tool) on
/clicked_point and save them to a YAML file for the patrol script.

Click order:
1) Kitchen
2) Bedroom
3) Home
"""

from __future__ import annotations

import os
from typing import List, Dict, Any

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

import yaml


DEFAULT_WAYPOINTS_PATH = "/home/dio/ros2_ws/config/patrol_waypoints.yaml"


class PatrolWaypointRecorder(Node):
    def __init__(self, output_path: str):
        super().__init__("patrol_waypoint_recorder")
        self._output_path = output_path
        self._clicks: List[PointStamped] = []

        self._sub = self.create_subscription(
            PointStamped,
            "/clicked_point",
            self._on_clicked_point,
            10,
        )

        self.get_logger().info("Patrol waypoint recorder running.")
        self.get_logger().info("Use RViz 'Publish Point' tool to click 3 points in order:")
        self.get_logger().info("1) Kitchen, 2) Bedroom, 3) Home")
        self.get_logger().info(f"Will write: {self._output_path}")

    def _on_clicked_point(self, msg: PointStamped) -> None:
        self._clicks.append(msg)
        idx = len(self._clicks)
        self.get_logger().info(
            f"Captured click {idx}/3: x={msg.point.x:.3f}, y={msg.point.y:.3f} (frame={msg.header.frame_id})"
        )

        if idx < 3:
            return

        try:
            self._write_yaml()
            self.get_logger().info("Saved patrol waypoints. Exiting recorder.")
        except Exception as e:
            self.get_logger().error(f"Failed to save waypoints: {e!r}")
        finally:
            rclpy.shutdown()

    def _write_yaml(self) -> None:
        kitchen, bedroom, home = self._clicks[:3]

        def p(msg: PointStamped) -> Dict[str, Any]:
            return {
                "frame_id": msg.header.frame_id or "map",
                "x": float(msg.point.x),
                "y": float(msg.point.y),
                "yaw": 0.0,
            }

        data = {
            "waypoints": {
                "kitchen": p(kitchen),
                "bedroom": p(bedroom),
                "home": p(home),
            }
        }

        out_dir = os.path.dirname(self._output_path)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)

        with open(self._output_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, sort_keys=False)


def main(args=None) -> None:
    rclpy.init(args=args)

    # Keep it simple: default path unless overridden by an environment variable.
    output_path = os.environ.get("PATROL_WAYPOINTS_PATH", DEFAULT_WAYPOINTS_PATH)
    node = PatrolWaypointRecorder(output_path=output_path)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

