#!/usr/bin/env python3
import os
import sys

import numpy as np
import rclpy
from scipy import ndimage
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def ground_mask(pts: np.ndarray, cell: float = 0.5, max_floor_z: float = 0.2,
                clearance: float = 0.2) -> np.ndarray:
    ix = np.floor(pts[:, 0] / cell).astype(np.int64)
    iy = np.floor(pts[:, 1] / cell).astype(np.int64)
    tile = (ix - ix.min()) * (iy.max() - iy.min() + 1) + (iy - iy.min())
    low = pts[:, 2] < max_floor_z
    floor = np.full(tile.max() + 1, -np.inf)
    order = np.argsort(tile[low], kind='stable')
    t_sorted, z_sorted = tile[low][order], pts[low, 2][order]
    starts = np.flatnonzero(np.r_[True, t_sorted[1:] != t_sorted[:-1]])
    for s, e in zip(starts, np.r_[starts[1:], len(t_sorted)]):
        floor[t_sorted[s]] = np.percentile(z_sorted[s:e], 5)
    return pts[:, 2] <= floor[tile] + clearance


def remove_ground(pts: np.ndarray, **kw) -> np.ndarray:
    return pts[~ground_mask(pts, **kw)]


def save_pcd(pts: np.ndarray, path: str, ascii_format: bool = True) -> int:
    pts = np.ascontiguousarray(pts, dtype=np.float32)
    n = pts.shape[0]
    header = (
        '# .PCD v0.7 - Point Cloud Data file format\n'
        'VERSION 0.7\n'
        'FIELDS x y z\n'
        'SIZE 4 4 4\n'
        'TYPE F F F\n'
        'COUNT 1 1 1\n'
        f'WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
        f'POINTS {n}\nDATA {"ascii" if ascii_format else "binary"}\n')
    with open(path, 'wb') as f:
        f.write(header.encode('ascii'))
        if ascii_format:
            np.savetxt(f, pts, fmt='%.4f')
        else:
            f.write(pts.tobytes())
    return n


def _disk(radius: int) -> np.ndarray:
    r = np.arange(-radius, radius + 1)
    return r[:, None] ** 2 + r[None, :] ** 2 <= radius ** 2


def build_grid(pts: np.ndarray, res: float, max_floor_z: float, zmax: float,
               min_height: float = 0.3, min_blob: int = 12, max_hole_m2: float = 2.0,
               small_obj_m: float = 1.0, gap_m: float = 0.3):
    x0, y0 = pts[:, 0].min() - 1.0, pts[:, 1].min() - 1.0
    w = int(np.ceil((pts[:, 0].max() + 1.0 - x0) / res))
    h = int(np.ceil((pts[:, 1].max() + 1.0 - y0) / res))
    cx = np.clip(((pts[:, 0] - x0) / res).astype(int), 0, w - 1)
    cy = np.clip(((pts[:, 1] - y0) / res).astype(int), 0, h - 1)
    ground = ground_mask(pts, max_floor_z=max_floor_z)
    obstacle = ~ground_mask(pts, max_floor_z=max_floor_z, clearance=min_height) \
        & (pts[:, 2] <= zmax)

    occ = np.zeros((h, w), bool)
    occ[cy[obstacle], cx[obstacle]] = True
    occ = ndimage.binary_closing(occ, structure=np.ones((3, 3)), border_value=0)
    labels, n = ndimage.label(occ, structure=np.ones((3, 3)))
    sizes = ndimage.sum(occ, labels, np.arange(1, n + 1))
    occ &= np.r_[False, sizes >= min_blob][labels]
    labels, _ = ndimage.label(occ, structure=np.ones((3, 3)))
    pad = 4
    for i, sl in enumerate(ndimage.find_objects(labels), start=1):
        if sl is None or max(sl[0].stop - sl[0].start, sl[1].stop - sl[1].start) * res \
                > small_obj_m:
            continue
        blob = np.pad(labels[sl] == i, pad)
        blob = ndimage.binary_fill_holes(ndimage.binary_closing(blob, structure=_disk(3)))
        ys = slice(sl[0].start - pad, sl[0].stop + pad)
        xs = slice(sl[1].start - pad, sl[1].stop + pad)
        if ys.start >= 0 and xs.start >= 0 and ys.stop <= h and xs.stop <= w:
            occ[ys, xs] |= blob
    holes, n = ndimage.label(~occ)
    sizes = ndimage.sum(~occ, holes, np.arange(1, n + 1))
    small = np.r_[False, sizes <= max_hole_m2 / res ** 2]
    small[holes[0, :]] = small[holes[-1, :]] = False
    small[holes[:, 0]] = small[holes[:, -1]] = False
    occ |= small[holes]

    seen = np.zeros((h, w), bool)
    seen[cy[ground], cx[ground]] = True
    seen |= occ
    r = max(1, int(round(gap_m / res)))
    seen = ndimage.binary_closing(seen, structure=_disk(r), border_value=0)
    seen = ndimage.binary_fill_holes(seen)

    img = np.full((h, w), 205, dtype=np.uint8)
    img[seen] = 254
    img[occ] = 0
    return img, x0, y0


def build_grid_full(pts: np.ndarray, res: float, zmin: float, zmax: float):
    x0, y0 = pts[:, 0].min() - 1.0, pts[:, 1].min() - 1.0
    w = int(np.ceil((pts[:, 0].max() + 1.0 - x0) / res))
    h = int(np.ceil((pts[:, 1].max() + 1.0 - y0) / res))
    cx = np.clip(((pts[:, 0] - x0) / res).astype(int), 0, w - 1)
    cy = np.clip(((pts[:, 1] - y0) / res).astype(int), 0, h - 1)
    obstacle = (pts[:, 2] >= zmin) & (pts[:, 2] <= zmax)
    floor = pts[:, 2] < zmin
    img = np.full((h, w), 205, dtype=np.uint8)
    img[cy[floor], cx[floor]] = 254
    img[cy[obstacle], cx[obstacle]] = 0
    return img, x0, y0


def save_grid(pts: np.ndarray, pgm_path: str, yaml_path: str,
              res: float, zmin: float, zmax: float, filtered: bool = True):
    if filtered:
        img, x0, y0 = build_grid(pts, res, zmin, zmax)
    else:
        img, x0, y0 = build_grid_full(pts, res, zmin, zmax)
    h, w = img.shape
    img = np.flipud(img)
    with open(pgm_path, 'wb') as f:
        f.write(f'P5\n# drone_slam map_saver\n{w} {h}\n255\n'.encode('ascii'))
        f.write(img.tobytes())
    with open(yaml_path, 'w') as f:
        f.write(f'image: {os.path.basename(pgm_path)}\n'
                'mode: trinary\n'
                f'resolution: {res:.4f}\n'
                f'origin: [{x0:.4f}, {y0:.4f}, 0.0000]\n'
                'negate: 0\n'
                'occupied_thresh: 0.65\n'
                'free_thresh: 0.25\n')
    return w, h


class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver')
        self.out = self.declare_parameter('output_dir', '/workspace/maps/map').value
        self.timeout = self.declare_parameter('timeout', 15.0).value
        self.res = self.declare_parameter('grid_resolution', 0.1).value
        self.zmin = self.declare_parameter('obstacle_min_z', 0.2).value
        self.zmax = self.declare_parameter('obstacle_max_z', 3.0).value
        self.pcd_binary = self.declare_parameter('pcd_binary', False).value
        self.remove_ground = self.declare_parameter('remove_ground', True).value
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.cloud = None
        self.create_subscription(PointCloud2, '/cloud_map', self._on_cloud, qos)

    def _on_cloud(self, msg):
        self.cloud = msg

    def run(self) -> bool:
        os.makedirs(self.out, exist_ok=True)
        end = self.get_clock().now().nanoseconds + int(self.timeout * 1e9)
        while rclpy.ok() and self.cloud is None:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.get_clock().now().nanoseconds > end:
                break
        if self.cloud is None or self.cloud.width * self.cloud.height == 0:
            self.get_logger().error('No /cloud_map received - is RTAB-Map running?')
            return False
        pts = point_cloud2.read_points_numpy(self.cloud, field_names=['x', 'y', 'z'],
                                             skip_nans=True)
        ascii_format = not self.pcd_binary
        if self.remove_ground:
            n_all = save_pcd(pts, os.path.join(self.out, 'map_full.pcd'), ascii_format)
            n = save_pcd(remove_ground(pts, max_floor_z=self.zmin), os.path.join(
                self.out, 'map.pcd'), ascii_format)
            self.get_logger().info(
                f'Saved 3D map: {self.out}/map.pcd ({n} points, ground removed: '
                f'{n_all - n}) + map_full.pcd (with ground)')
        else:
            n = save_pcd(pts, os.path.join(self.out, 'map.pcd'), ascii_format)
            self.get_logger().info(f'Saved 3D map: {self.out}/map.pcd ({n} points)')
        w, h = save_grid(pts, os.path.join(self.out, 'map.pgm'),
                         os.path.join(self.out, 'map.yaml'), self.res, self.zmin, self.zmax,
                         filtered=self.remove_ground)
        extra = ''
        if self.remove_ground:
            save_grid(pts, os.path.join(self.out, 'map_full.pgm'),
                      os.path.join(self.out, 'map_full.yaml'), self.res, self.zmin, self.zmax,
                      filtered=False)
            extra = ' (floor filtered) + map_full.pgm/yaml (unfiltered)'
        self.get_logger().info(
            f'Saved 2D map: {self.out}/map.pgm + map.yaml{extra} ({w}x{h} @ {self.res:.2f} m)')
        return True


def main():
    rclpy.init()
    node = MapSaver()
    ok = node.run()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
