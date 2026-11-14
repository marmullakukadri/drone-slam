#!/usr/bin/env python3
"""Floor-filtered copies of the live RTAB-Map map, for RViz.

  /cloud_map  ->  /cloud_map_filtered   3D map without the floor (= map.pcd)
              ->  /map_filtered         clean 2D occupancy grid    (= map.pgm)

Same filtering as map_saver.py (ground_mask / build_grid), so RViz shows what
"task ruaj" saves. Both outputs are latched (transient local). The planner in
drone_node keeps using /cloud_map.
"""
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
        self.period = self.declare_parameter('min_period', 2.0).value  # s between updates
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.cloud_pub = self.create_publisher(PointCloud2, 'cloud_map_filtered', qos)
        self.grid_pub = self.create_publisher(OccupancyGrid, 'map_filtered', qos)
        self.create_subscription(PointCloud2, 'cloud_map', self.on_cloud, qos)
        self.pending, self.last = None, 0.0
        self.create_timer(0.5, self.process)

    def on_cloud(self, msg):
        self.pending = msg          # keep only the newest map

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
        data = np.full(img.shape, -1, dtype=np.int8)     # 205 = unknown
        data[img == 254] = 0                             # free
        data[img == 0] = 100                             # occupied
        grid.data = data.ravel().tolist()                # row 0 = min y, as build_grid
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
        except KeyboardInterrupt:   # second Ctrl+C / SIGINT from "task ndal"
            pass


if __name__ == '__main__':
    main()
