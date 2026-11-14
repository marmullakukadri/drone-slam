#!/usr/bin/env python3
"""Compare the SLAM pose (TF map->base_link) against Gazebo ground truth.

Ground truth arrives on /odom_gt in the Gazebo 'world' frame and is converted
into the SLAM 'map' frame with the static TF map->world published by the
launch file (map origin = where the drone started while the map was built).

Publishes:
  /slam/position_error  std_msgs/Float64   instantaneous 3D error [m]
Logs a running ATE RMSE and writes a CSV to <output_dir> on shutdown.
"""
import bisect
import csv
import math
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformException, TransformListener


class SlamEvaluator(Node):
    def __init__(self):
        super().__init__('slam_evaluator')
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.output_dir = self.declare_parameter('output_dir', '/workspace/results').value
        self.sample_period = self.declare_parameter('sample_period', 0.2).value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.err_pub = self.create_publisher(Float64, '/slam/position_error', 10)
        self.create_subscription(Odometry, '/odom_gt', self.on_gt, 50)
        # A new initial pose (RViz "2D Pose Estimate") restarts the statistics:
        # errors before it only measure the wrong starting guess.
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose',
                                 self.on_initial_pose, 1)
        self.create_timer(self.sample_period, self.sample)
        self.create_timer(10.0, self.report)

        self.map_T_world = None  # (translation, quaternion) from TF, static
        self.gt_t = []    # ground-truth history (stamps, seconds)
        self.gt_p = []    # matching positions in the 'world' frame
        self.last_sample = -1.0
        self.rows = []  # t, gt_x, gt_y, gt_z, est_x, est_y, est_z, err
        self.csv_path = os.path.join(self.output_dir,
                                     time.strftime('slam_eval_%Y%m%d_%H%M%S.csv'))

    def on_initial_pose(self, _msg):
        self.get_logger().info('Initial pose received - restarting SLAM error statistics')
        self.save(quiet=True)
        self.rows = []
        self.csv_path = os.path.join(self.output_dir,
                                     time.strftime('slam_eval_%Y%m%d_%H%M%S_loc.csv'))

    def on_gt(self, msg: Odometry):
        p = msg.pose.pose.position
        self.gt_frame = msg.header.frame_id
        self.gt_t.append(Time.from_msg(msg.header.stamp).nanoseconds * 1e-9)
        self.gt_p.append((p.x, p.y, p.z))
        if len(self.gt_t) > 5000:  # keep ~100 s at 50 Hz
            del self.gt_t[:1000]
            del self.gt_p[:1000]

    def sample(self):
        """Match the newest SLAM pose with the ground truth at the same stamp."""
        if not self.gt_t:
            return
        try:
            if self.map_T_world is None:
                s = self.tf_buffer.lookup_transform(self.map_frame, self.gt_frame, Time())
                self.map_T_world = (s.transform.translation, s.transform.rotation)
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except TransformException:
            return  # SLAM not ready yet
        t = Time.from_msg(tf.header.stamp).nanoseconds * 1e-9
        if t <= self.last_sample:
            return
        i = bisect.bisect_left(self.gt_t, t)
        if i == 0 or i >= len(self.gt_t):
            return
        t0, t1 = self.gt_t[i - 1], self.gt_t[i]
        a = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        gt_world = [p0 + a * (p1 - p0) for p0, p1 in zip(self.gt_p[i - 1], self.gt_p[i])]
        gt = self._to_map(gt_world)
        self.last_sample = t
        tr = tf.transform.translation
        est = (tr.x, tr.y, tr.z)
        err = math.dist(gt, est)
        self.rows.append((t, *gt, *est, err))
        self.err_pub.publish(Float64(data=err))

    def _to_map(self, p):
        """Apply the static map->world transform to a point given in 'world'."""
        tr, q = self.map_T_world
        # v' = q * v * q^-1  (rotation of p by unit quaternion q)
        ux, uy, uz, w = q.x, q.y, q.z, q.w
        x, y, z = p
        tx, ty, tz = 2 * (uy * z - uz * y), 2 * (uz * x - ux * z), 2 * (ux * y - uy * x)
        rx = x + w * tx + (uy * tz - uz * ty)
        ry = y + w * ty + (uz * tx - ux * tz)
        rz = z + w * tz + (ux * ty - uy * tx)
        return (rx + tr.x, ry + tr.y, rz + tr.z)

    def stats(self):
        if not self.rows:
            return None
        errs = [r[-1] for r in self.rows]
        rmse = math.sqrt(sum(e * e for e in errs) / len(errs))
        return rmse, max(errs), errs[-1], len(errs)

    def report(self):
        s = self.stats()
        if s is None:
            self.get_logger().info('Waiting for SLAM pose (TF %s->%s)...'
                                   % (self.map_frame, self.base_frame))
            return
        self.get_logger().info('SLAM ATE RMSE %.3f m | max %.3f m | current %.3f m | %d samples' % s)
        self.save(quiet=True)  # keep the CSV up to date even if killed

    def save(self, quiet=False):
        if not self.rows:
            return
        os.makedirs(self.output_dir, exist_ok=True)
        path = self.csv_path
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'gt_x', 'gt_y', 'gt_z', 'est_x', 'est_y', 'est_z', 'error'])
            w.writerows(self.rows)
        if quiet:
            return
        rmse, mx, _, n = self.stats()
        print(f'[slam_evaluator] saved {n} samples to {path} (RMSE {rmse:.3f} m, max {mx:.3f} m)')


def main():
    rclpy.init()
    node = SlamEvaluator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
