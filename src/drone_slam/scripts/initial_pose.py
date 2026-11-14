#!/usr/bin/env python3
"""Give RTAB-Map the start pose automatically (localization mode).

The drone always starts at a known place (spawn_x / spawn_y / spawn_yaw in the
map frame), so there is no need to click "2D Pose Estimate": this node waits
until RTAB-Map has loaded the map and odometry is running, publishes
/initialpose, checks with TF that SLAM put base_link there, retries if not,
and exits. "2D Pose Estimate" in RViz still works for manual corrections.
"""
import math
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2

TOLERANCE = 0.3   # m, SLAM pose must end up this close to the start pose
TRIES = 5


class InitialPose(Node):
    def __init__(self):
        super().__init__('initial_pose')
        self.x = self.declare_parameter('x', 0.0).value
        self.y = self.declare_parameter('y', 0.0).value
        self.yaw = self.declare_parameter('yaw', 0.0).value
        self.frame = self.declare_parameter('map_frame', 'map').value
        self.base = self.declare_parameter('base_frame', 'base_link').value
        self.pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 1)
        self.tf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf, self)
        self.have_map = False
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PointCloud2, 'cloud_map', self._on_map, qos)

    def _on_map(self, _msg):
        self.have_map = True

    def spin_for(self, sec):
        end = time.monotonic() + sec
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def slam_xy(self):
        try:
            t = self.tf.lookup_transform(self.frame, self.base, Time(),
                                         timeout=Duration(seconds=0.5)).transform.translation
            return t.x, t.y
        except tf2_ros.TransformException:
            return None

    def run(self):
        # RTAB-Map loaded the map, listens on /initialpose and base_link exists
        start = time.monotonic()
        while rclpy.ok() and (not self.have_map or self.pub.get_subscription_count() == 0
                              or self.slam_xy() is None):
            if time.monotonic() - start > 120:
                self.get_logger().error('RTAB-Map not ready after 120 s - give "2D Pose Estimate"')
                return
            self.spin_for(0.5)
        self.spin_for(2.0)

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.frame
        msg.pose.pose.position.x, msg.pose.pose.position.y = self.x, self.y
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2)
        for attempt in range(1, TRIES + 1):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(msg)
            self.spin_for(3.0)
            xy = self.slam_xy()
            err = math.hypot(xy[0] - self.x, xy[1] - self.y) if xy else float('inf')
            if err < TOLERANCE:
                self.get_logger().info(
                    f'Initial pose set: x={self.x:.2f} y={self.y:.2f} '
                    f'yaw={math.degrees(self.yaw):.0f} deg (SLAM {err:.2f} m from it)')
                return
            self.get_logger().warn(f'Initial pose attempt {attempt}: SLAM is {err:.2f} m off')
        self.get_logger().error('Initial pose not accepted - give "2D Pose Estimate" in RViz')


def main():
    rclpy.init()
    node = InitialPose()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
