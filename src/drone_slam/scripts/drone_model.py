#!/usr/bin/env python3
"""Show the Gazebo drone in RViz, with spinning propellers.

Reads the visuals (box / cylinder / sphere) of models/slam_drone/model.sdf and
publishes them as markers:

  /drone_model     the drone where SLAM thinks it is (base_link frame)
  /drone_model_gt  a translucent cyan "ghost" where the drone really is in
                   Gazebo (/odom_gt). Give RViz "2D Pose Estimate" on the ghost;
                   while flying, the further apart the two are, the larger the
                   SLAM error.

The propeller links turn while drone_node spins them in Gazebo (/drone/prop_speed).
"""
import math
import os
import xml.etree.ElementTree as ET

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import ColorRGBA, Float64
from visualization_msgs.msg import Marker, MarkerArray

STATIC_PERIOD = 1.0     # s, body / arms / motors (frame_locked: follow TF in between)
PROP_PERIOD = 1 / 30.0  # s, propellers
GHOST_PERIOD = 1 / 15.0  # s, ground-truth ghost
MAX_SHOWN_SPIN = 15.0   # rad/s - faster turns alias into a jitter at 30 fps
GHOST_COLOR = ColorRGBA(r=0.2, g=0.9, b=1.0, a=0.5)
GHOST_LABEL_COLOR = ColorRGBA(r=0.2, g=0.9, b=1.0, a=1.0)
SLAM_COLOR = ColorRGBA(r=1.0, g=0.55, b=0.1, a=1.0)
LABEL_HEIGHT = 0.6      # m, text size


def _pose6(text):
    v = [float(x) for x in (text or '').split()] if text else []
    return (v + [0.0] * 6)[:6]


def _quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def _rotate(q, v):
    x, y, z, w = _qmul(_qmul(q, (v[0], v[1], v[2], 0.0)), (-q[0], -q[1], -q[2], q[3]))
    return x, y, z


def _visual_marker(vis):
    """Marker (without header / id / pose) for one SDF <visual>, or None."""
    geo = vis.find('geometry')
    m = Marker()
    if geo.find('box') is not None:
        m.type = Marker.CUBE
        m.scale.x, m.scale.y, m.scale.z = map(float, geo.find('box/size').text.split())
    elif geo.find('cylinder') is not None:
        r = float(geo.find('cylinder/radius').text)
        m.type = Marker.CYLINDER
        m.scale.x = m.scale.y = 2 * r
        m.scale.z = float(geo.find('cylinder/length').text)
    elif geo.find('sphere') is not None:
        m.type = Marker.SPHERE
        m.scale.x = m.scale.y = m.scale.z = 2 * float(geo.find('sphere/radius').text)
    else:
        return None
    rgb = (vis.findtext('material/diffuse') or vis.findtext('material/ambient') or '0.5 0.5 0.5')
    r, g, b = [float(x) for x in rgb.split()[:3]]
    alpha = 1.0 - float(vis.findtext('transparency') or 0.0)
    m.color = ColorRGBA(r=r, g=g, b=b, a=alpha)
    return m


class DroneModel(Node):
    def __init__(self):
        super().__init__('drone_model')
        ws = self.declare_parameter('workspace', '/workspace').value
        sdf = self.declare_parameter(
            'model_file', os.path.join(ws, 'models', 'slam_drone', 'model.sdf')).value
        self.frame = self.declare_parameter('frame_id', 'base_link').value

        model = ET.parse(sdf).getroot().find('model')
        self.static = []      # (marker, visual pose)
        self.props = []       # (marker, link pose, visual pose, axis sign)
        joints = {j.findtext('child'): j for j in model.findall('joint')}
        for link in model.findall('link'):
            name = link.get('name')
            for vis in link.findall('visual'):
                m = _visual_marker(vis)
                if m is None:
                    continue
                vpose = _pose6(vis.findtext('pose'))
                if name in joints:            # spinning propeller link
                    axis = joints[name].findtext('axis/xyz') or '0 0 1'
                    sign = 1.0 if float(axis.split()[2]) >= 0 else -1.0
                    self.props.append((m, _pose6(link.findtext('pose')), vpose, sign))
                else:
                    self.static.append((m, vpose))
        self.get_logger().info(
            f'Drone model from {sdf}: {len(self.static)} parts + {len(self.props)} propeller parts')

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, 'drone_model', qos)
        self.gt_pub = self.create_publisher(MarkerArray, 'drone_model_gt', qos)
        self.spin_speed, self.angle, self.gt = 0.0, 0.0, None
        self.create_subscription(Float64, '/drone/prop_speed', self.on_speed, 10)
        self.create_subscription(Odometry, '/odom_gt', self.on_gt, 10)
        self.create_timer(STATIC_PERIOD, self.publish_static)
        self.create_timer(PROP_PERIOD, self.publish_props)
        self.create_timer(GHOST_PERIOD, self.publish_ghost)
        self.publish_static()

    def on_speed(self, msg):
        self.spin_speed = max(-MAX_SHOWN_SPIN, min(MAX_SHOWN_SPIN, msg.data))

    def on_gt(self, msg):
        self.gt = msg

    # ---- parts in the drone body frame: (template, ns, id, (x, y, z), quaternion)
    def body_parts(self):
        for i, (t, p) in enumerate(self.static):
            yield t, 'body', i, (p[0], p[1], p[2]), _quat(p[3], p[4], p[5])

    def prop_parts(self):
        for i, (t, lp, vp, sign) in enumerate(self.props):
            yaw = lp[5] + sign * self.angle           # link turns about its own z
            c, s = math.cos(yaw), math.sin(yaw)
            xyz = (lp[0] + c * vp[0] - s * vp[1], lp[1] + s * vp[0] + c * vp[1], lp[2] + vp[2])
            yield t, 'propellers', i, xyz, _quat(vp[3], vp[4], yaw + vp[5])

    def _marker(self, template, ns, mid, frame, stamp, xyz, q, color=None):
        m = Marker()
        # real (sim) stamp: RViz waits for the TF instead of failing with
        # "No transform" when it starts before SLAM publishes base_link
        m.header.frame_id, m.header.stamp = frame, stamp
        m.ns, m.id, m.action = ns, mid, Marker.ADD
        m.type, m.scale, m.text = template.type, template.scale, template.text
        m.color = color or template.color
        m.frame_locked = True
        m.pose.position.x, m.pose.position.y, m.pose.position.z = xyz
        m.pose.orientation.x, m.pose.orientation.y = q[0], q[1]
        m.pose.orientation.z, m.pose.orientation.w = q[2], q[3]
        return m

    @staticmethod
    def _shape(mtype, sx, sy, sz, text=''):
        m = Marker()
        m.type, m.text = mtype, text
        m.scale.x, m.scale.y, m.scale.z = sx, sy, sz
        return m

    def publish_static(self):
        stamp = self.get_clock().now().to_msg()
        markers = [self._marker(t, ns, i, self.frame, stamp, xyz, q)
                   for t, ns, i, xyz, q in self.body_parts()]
        label = self._shape(Marker.TEXT_VIEW_FACING, 0.0, 0.0, LABEL_HEIGHT, 'SLAM')
        markers.append(self._marker(label, 'label', 0, self.frame, stamp, (0.0, 0.0, 0.8),
                                    (0.0, 0.0, 0.0, 1.0), SLAM_COLOR))
        self.pub.publish(MarkerArray(markers=markers))

    def publish_props(self):
        self.angle = (self.angle + self.spin_speed * PROP_PERIOD) % (2 * math.pi)
        stamp = self.get_clock().now().to_msg()
        self.pub.publish(MarkerArray(markers=[
            self._marker(t, ns, i, self.frame, stamp, xyz, q)
            for t, ns, i, xyz, q in self.prop_parts()]))

    def publish_ghost(self):
        if self.gt is None:
            return
        p, o = self.gt.pose.pose.position, self.gt.pose.pose.orientation
        qd = (o.x, o.y, o.z, o.w)
        markers = []
        for t, ns, i, xyz, q in list(self.body_parts()) + list(self.prop_parts()):
            r = _rotate(qd, xyz)
            markers.append(self._marker(
                t, 'ghost_' + ns, i, self.gt.header.frame_id, self.gt.header.stamp,
                (p.x + r[0], p.y + r[1], p.z + r[2]), _qmul(qd, q), GHOST_COLOR))
        frame, stamp, up = self.gt.header.frame_id, self.gt.header.stamp, (0.0, 0.0, 0.0, 1.0)
        label = self._shape(Marker.TEXT_VIEW_FACING, 0.0, 0.0, LABEL_HEIGHT, 'Droni (Gazebo)')
        markers.append(self._marker(label, 'ghost_label', 0, frame, stamp,
                                    (p.x, p.y, 4.6), up, GHOST_LABEL_COLOR))  # above 4 m walls
        self.gt_pub.publish(MarkerArray(markers=markers))


def main():
    rclpy.init()
    node = DroneModel()
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
