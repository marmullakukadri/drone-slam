#!/usr/bin/env python3
import time

import numpy as np
import rclpy
from map_saver import build_grid, remove_ground
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class MapFilter(Node):
    def __init__(self):
        super().__init__('map_filter')
        self.res = self.declare_parameter('grid_resolution', 0.1).value
        self.zmin = self.declare_parameter('obstacle_min_z', 0.2).value
        self.zmax = self.declare_parameter('obstacle_max_z', 3.0).value
        self.period = self.declare_parameter('min_period', 2.0).value
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.cloud_pub = self.create_publisher(PointCloud2, 'cloud_map_filtered', qos)
        self.grid_pub = self.create_publisher(OccupancyGrid, 'map_filtered', qos)
        self.create_subscription(PointCloud2, 'cloud_map', self.on_cloud, qos)
        self.pending, self.last = None, 0.0
        self.create_timer(0.5, self.process)

    def on_cloud(self, msg):
        self.pending = msg

    def process(self):
        if self.pending is None or time.monotonic() - self.last < self.period:
            return
        msg, self.pending, self.last = self.pending, None, time.monotonic()
        pts = point_cloud2.read_points_numpy(msg, field_names=['x', 'y', 'z'], skip_nans=True)
        if len(pts) < 100:
            return
        pts = np.ascontiguousarray(pts, dtype=np.float32)

        kept = remove_ground(pts, max_floor_z=self.zmin)
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(msg.header, kept))

        img, x0, y0 = build_grid(pts, self.res, self.zmin, self.zmax)
        grid = OccupancyGrid()
        grid.header = msg.header
        grid.info.resolution = float(self.res)
        grid.info.height, grid.info.width = img.shape
        grid.info.origin.position.x, grid.info.origin.position.y = float(x0), float(y0)
        grid.info.origin.orientation.w = 1.0
        data = np.full(img.shape, -1, dtype=np.int8)
        data[img == 254] = 0
        data[img == 0] = 100
        grid.data = data.ravel().tolist()
        self.grid_pub.publish(grid)


def main():
    rclpy.init()
    node = MapFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
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
