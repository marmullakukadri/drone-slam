#!/usr/bin/env python3
"""Keyboard control for the SLAM drone - several keys at the same time.

Opens a small window on the virtual desktop (VNC, DISPLAY=:1). Click it once,
then hold any combination of keys, e.g. ↑ + q = fly forward while turning left.

    ↑ / ↓      forward / backward
    ← / →      left / right (sideways)
    u / d      up / down
    q / e      rotate left / right
    + / -      faster / slower
    space      stop

The drone moves while keys are held and stops when they are released (or when
the window loses focus). A terminal cannot do this: it only reports the last
key that auto-repeats and never reports releases, so this uses real X11
KeyPress / KeyRelease events.

Publishes body-frame geometry_msgs/Twist on /cmd_vel_teleop, which drone_node
gives priority over goals and missions while keys are pressed.
"""
import math
import sys
import tkinter as tk

import rclpy
from geometry_msgs.msg import Twist

TITLE = 'Tastiera e dronit'
RATE_MS = 50           # publish every 50 ms (20 Hz)
RELEASE_DELAY_MS = 60  # X auto-repeat sends release+press pairs; ignore those

# keysym -> (axis, sign); axis: vx, vy, vz, wz
BINDINGS = {
    'Up': ('vx', 1), 'Down': ('vx', -1),
    'Left': ('vy', 1), 'Right': ('vy', -1),
    'u': ('vz', 1), 'd': ('vz', -1),
    'q': ('wz', 1), 'e': ('wz', -1),
}
# on-screen keys: (keysym, label, row, column)
LAYOUT = [
    ('q', 'q\n⟲', 0, 0), ('Up', '↑', 0, 1), ('e', 'e\n⟳', 0, 2), ('u', 'u\nlart', 0, 4),
    ('Left', '←', 1, 0), ('Down', '↓', 1, 1), ('Right', '→', 1, 2), ('d', 'd\nposhtë', 1, 4),
]
NAMES = {('vx', 1): 'para', ('vx', -1): 'mbrapa', ('vy', 1): 'majtas', ('vy', -1): 'djathtas',
         ('vz', 1): 'lart', ('vz', -1): 'poshtë', ('wz', 1): 'rrotullim majtas',
         ('wz', -1): 'rrotullim djathtas'}
IDLE_BG, ACTIVE_BG = '#3a3f47', '#2e9d5b'


class Teleop:
    def __init__(self, node, root):
        self.node, self.root = node, root
        self.speed = node.declare_parameter('speed', 0.6).value          # m/s horizontal
        self.vz_speed = node.declare_parameter('vertical_speed', 0.3).value
        # rad/s - slow on purpose: lidar odometry updates ~3x/s and loses the
        # heading (the map comes out rotated / doubled) when turning faster
        self.yaw_speed = node.declare_parameter('yaw_speed', 0.4).value
        self.pub = node.create_publisher(Twist, '/cmd_vel_teleop', 10)
        self.held = set()          # keysyms currently held down
        self.pending = {}          # keysym -> after() id of a delayed release
        self.zeros_left = 0

        root.title(TITLE)
        root.configure(bg='#22252a', padx=10, pady=8)
        root.resizable(False, False)
        root.attributes('-topmost', True)

        self.tiles = {}
        grid = tk.Frame(root, bg='#22252a')
        grid.pack()
        for sym, label, r, c in LAYOUT:
            t = tk.Label(grid, text=label, width=6, height=2, bg=IDLE_BG, fg='white',
                         font=('DejaVu Sans', 11, 'bold'))
            t.grid(row=r, column=c, padx=3, pady=3)
            self.tiles[sym] = t
        tk.Frame(grid, width=14, bg='#22252a').grid(row=0, column=3)

        self.status = tk.Label(root, bg='#22252a', fg='#e8e8e8', font=('DejaVu Sans', 10),
                               justify='left', anchor='w', width=40)
        self.status.pack(pady=(6, 0), fill='x')
        tk.Label(root, text='Mbani disa taste njëkohësisht · +/- shpejtësia · hapësirë = ndal\n'
                            'Klikoni këtu nëse tastet nuk reagojnë',
                 bg='#22252a', fg='#9aa0a6', font=('DejaVu Sans', 8), justify='left').pack(
            pady=(4, 0), fill='x')

        root.bind('<KeyPress>', self.on_press)
        root.bind('<KeyRelease>', self.on_release)
        root.bind('<FocusOut>', lambda _e: self.release_all())
        root.bind('<Button-1>', lambda _e: root.focus_force())
        root.protocol('WM_DELETE_WINDOW', self.close)
        root.after(300, root.focus_force)
        self.update_view()
        root.after(RATE_MS, self.tick)

    # ------------------------------------------------------------- keyboard
    def on_press(self, e):
        sym = e.keysym
        if sym in ('U', 'D', 'Q', 'E'):
            sym = sym.lower()
        if sym in self.pending:                       # auto-repeat: key is still held
            self.root.after_cancel(self.pending.pop(sym))
        if sym in BINDINGS:
            if sym not in self.held:
                self.held.add(sym)
                self.update_view()
        elif sym in ('plus', 'equal', 'KP_Add'):
            self.speed = min(2.0, self.speed + 0.1)
            self.vz_speed = min(1.0, self.vz_speed + 0.05)
            self.update_view()
        elif sym in ('minus', 'KP_Subtract'):
            self.speed = max(0.1, self.speed - 0.1)
            self.vz_speed = max(0.1, self.vz_speed - 0.05)
            self.update_view()
        elif sym == 'space':
            self.release_all()
        elif sym == 'c' and e.state & 0x4:            # Ctrl+C
            self.close()

    def on_release(self, e):
        sym = e.keysym.lower() if e.keysym in ('U', 'D', 'Q', 'E') else e.keysym
        if sym in self.held and sym not in self.pending:
            self.pending[sym] = self.root.after(RELEASE_DELAY_MS, self.release, sym)

    def release(self, sym):
        self.pending.pop(sym, None)
        if sym in self.held:
            self.held.discard(sym)
            if not self.held:
                self.zeros_left = 5
            self.update_view()

    def release_all(self):
        for aid in self.pending.values():
            self.root.after_cancel(aid)
        self.pending.clear()
        if self.held:
            self.held.clear()
            self.update_view()
        self.zeros_left = 5

    # ------------------------------------------------------------- output
    def command(self):
        axes = {'vx': 0, 'vy': 0, 'vz': 0, 'wz': 0}
        for sym in self.held:
            axis, sign = BINDINGS[sym]
            axes[axis] += sign          # opposite keys cancel out
        msg = Twist()
        norm = math.hypot(axes['vx'], axes['vy'])
        if norm > 0:                    # diagonal is not faster than straight
            msg.linear.x = axes['vx'] / norm * self.speed
            msg.linear.y = axes['vy'] / norm * self.speed
        msg.linear.z = axes['vz'] * self.vz_speed
        msg.angular.z = axes['wz'] * self.yaw_speed
        return msg

    def tick(self):
        if not rclpy.ok():
            self.root.destroy()
            return
        if self.held:
            self.pub.publish(self.command())
        elif self.zeros_left > 0:       # a few zeros so the drone stops cleanly
            self.pub.publish(Twist())
            self.zeros_left -= 1
        self.root.after(RATE_MS, self.tick)

    def update_view(self):
        for sym, tile in self.tiles.items():
            tile.configure(bg=ACTIVE_BG if sym in self.held else IDLE_BG)
        moves = [NAMES[BINDINGS[s]] for s in BINDINGS if s in self.held]
        self.status.configure(
            text=f'{" + ".join(moves) if moves else "pezull"}\n'
                 f'shpejtësia {self.speed:.1f} m/s   lart/poshtë {self.vz_speed:.2f} m/s')

    def close(self):
        self.pub.publish(Twist())
        self.root.destroy()


def main():
    rclpy.init()
    node = rclpy.create_node('drone_teleop')
    try:
        root = tk.Tk(className='DroneTeleop')
    except tk.TclError as err:
        print(f'drone_teleop needs a display (DISPLAY=:1 on the VNC desktop): {err}')
        node.destroy_node()
        rclpy.shutdown()
        return 1
    app = Teleop(node, root)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        app.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
