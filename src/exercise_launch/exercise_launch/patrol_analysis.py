#!/usr/bin/env python3
"""
Phase 6 - Patrol playback analysis helpers.

Subscribes:
- /plan (nav_msgs/Path): planned global path (map frame)
- /odometry/filtered (nav_msgs/Odometry): actual robot motion (odom frame)

Uses TF from the bag (map <-> odom) to transform actual motion into map frame.

Publishes:
- /planned_path (nav_msgs/Path): copy of /plan
- /actual_path (nav_msgs/Path): accumulated actual path in map frame
- /planned_xy (geometry_msgs/PointStamped): planned points streamed for PlotJuggler
- /actual_xy (geometry_msgs/PointStamped): actual points streamed for PlotJuggler
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path

import tf2_ros
from tf2_ros import TransformException

from tf2_geometry_msgs import do_transform_pose  # type: ignore


@dataclass
class _PlanStream:
    frame_id: str
    points: List[PointStamped]
    idx: int = 0


class PatrolAnalysis(Node):
    def __init__(self):
        super().__init__("patrol_analysis")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("max_actual_path_poses", 5000)
        self.declare_parameter("planned_stream_hz", 20.0)

        self._map_frame = str(self.get_parameter("map_frame").value)
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._max_actual_path_poses = int(self.get_parameter("max_actual_path_poses").value)
        self._planned_stream_hz = float(self.get_parameter("planned_stream_hz").value)

        self._tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._planned_path_pub = self.create_publisher(Path, "/planned_path", 10)
        self._actual_path_pub = self.create_publisher(Path, "/actual_path", 10)
        self._planned_xy_pub = self.create_publisher(PointStamped, "/planned_xy", 10)
        self._actual_xy_pub = self.create_publisher(PointStamped, "/actual_xy", 50)

        self._plan_sub = self.create_subscription(Path, "/plan", self._on_plan, 10)
        self._odom_sub = self.create_subscription(Odometry, "/odometry/filtered", self._on_odom, 50)

        self._actual_path = Path()
        self._actual_path.header.frame_id = self._map_frame

        self._plan_stream: Optional[_PlanStream] = None
        self._plan_timer = self.create_timer(
            1.0 / max(self._planned_stream_hz, 1e-3),
            self._tick_plan_stream,
        )

        self.get_logger().info(
            f"Patrol analysis running. map={self._map_frame} odom={self._odom_frame} base={self._base_frame}"
        )

    def _on_plan(self, msg: Path) -> None:
        # Republish /plan as /planned_path (normalize frame to map if missing).
        planned = Path()
        planned.header.stamp = msg.header.stamp
        planned.header.frame_id = msg.header.frame_id or self._map_frame
        planned.poses = msg.poses
        self._planned_path_pub.publish(planned)

        # Build a point stream for PlotJuggler (loop through path points).
        points: List[PointStamped] = []
        frame_id = planned.header.frame_id
        for ps in planned.poses:
            pt = PointStamped()
            pt.header.stamp = ps.header.stamp
            pt.header.frame_id = frame_id
            pt.point.x = ps.pose.position.x
            pt.point.y = ps.pose.position.y
            pt.point.z = 0.0
            points.append(pt)

        if points:
            self._plan_stream = _PlanStream(frame_id=frame_id, points=points, idx=0)

    def _tick_plan_stream(self) -> None:
        if self._plan_stream is None or not self._plan_stream.points:
            return
        # Publish one planned point per tick (wrap around).
        pt = self._plan_stream.points[self._plan_stream.idx]
        pt.header.stamp = self.get_clock().now().to_msg()
        self._planned_xy_pub.publish(pt)
        self._plan_stream.idx = (self._plan_stream.idx + 1) % len(self._plan_stream.points)

    def _on_odom(self, msg: Odometry) -> None:
        # Convert Odometry -> PoseStamped in odom frame
        pose_odom = PoseStamped()
        pose_odom.header = msg.header
        pose_odom.pose = msg.pose.pose

        # Transform to map frame using TF from bag
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame,
                pose_odom.header.frame_id or self._odom_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except TransformException:
            return

        # Fix: do_transform_pose in some ROS2 versions expects the .pose member, 
        # but in Jazzy we should use the registration-aware transform.
        try:
            from tf2_geometry_msgs import do_transform
            pose_map = do_transform(pose_odom, tf)
        except Exception:
            # Fallback for manual transform if registration fails
            pose_map = do_transform_pose(pose_odom.pose, tf)
            # Re-wrap in PoseStamped if fallback was used
            new_ps = PoseStamped()
            new_ps.header.frame_id = self._map_frame
            new_ps.header.stamp = pose_odom.header.stamp
            new_ps.pose = pose_map
            pose_map = new_ps

        pose_map.header.stamp = self.get_clock().now().to_msg()
        pose_map.header.frame_id = self._map_frame

        # Publish actual_xy stream for PlotJuggler
        pt = PointStamped()
        pt.header = pose_map.header
        pt.point.x = pose_map.pose.position.x
        pt.point.y = pose_map.pose.position.y
        pt.point.z = 0.0
        self._actual_xy_pub.publish(pt)

        # Append to /actual_path
        self._actual_path.header.stamp = pose_map.header.stamp
        self._actual_path.poses.append(pose_map)
        if len(self._actual_path.poses) > self._max_actual_path_poses:
            self._actual_path.poses = self._actual_path.poses[-self._max_actual_path_poses :]
        self._actual_path_pub.publish(self._actual_path)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PatrolAnalysis()
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

