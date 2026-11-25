#!/usr/bin/env python3
"""Cho xe TU CHAY toi dich (khong teleop), dung chinh xac, in ket qua de so voi thuoc do.

Vi tri: amm_base_description/scripts/goto_test.py
Can: real_base.launch.py use_ekf:=true (phan hoi vi tri + huong tu /odometry/filtered).
TAT teleop truoc khi chay (2 nguon lenh se tranh nhau).

MOC TOA DO = vi tri + huong xe LUC CHAY LENH.  x = tien, y = sang TRAI, yaw = xoay trai (+).

LENH
  ros2 run amm_base_description goto_test.py goto 1.0 0 0        di toi (x=1 m, y=0), huong 0°
  ros2 run amm_base_description goto_test.py goto 0 0.5 90       sang trai 0.5 m, xoay trai 90°
  ros2 run amm_base_description goto_test.py line 1.0             tien 1 m, dung, lui ve 0
  ros2 run amm_base_description goto_test.py turn 360             xoay tai cho 360° (am = xoay phai)
  ros2 run amm_base_description goto_test.py square 1.0 ccw       hinh vuong 1 m, CO xoay o goc (UMBmark)
  ros2 run amm_base_description goto_test.py square 1.0 cw
  ros2 run amm_base_description goto_test.py square_holo 0.5     hinh vuong KHONG xoay (tien, ngang, lui, ngang)

  Tuy chon (dat SAU lenh): --vmax 0.06  --wmax 0.3  --tol_mm 5  --tol_deg 0.5  --pose ekf|odom  --pause 1.0

CACH DO KET QUA
  Dan bang keo danh dau GOC TRUOC-TRAI va GOC SAU-TRAI cua khung luc xuat phat.
  Sau moi dich (hoac cuoi bai), node in vi tri EKF va odom. Do vi tri THAT cua 2 goc do bang thuoc.
  Cac bai quay ve diem xuat phat (line, square): sai lech that = khoang cach tu 2 goc luc dau toi luc cuoi.

  Node dieu khien vong kin theo EKF, nen "sai so dieu khien" (EKF so voi dich) luon nho (~mm).
  Sai lech THAT do bang thuoc = sai so DINH VI (odom + IMU). Day moi la con so can kiem chung.

LOG: ~/goto_log_<thoi gian>.csv (t, dich, EKF x y yaw, odom x y yaw, lenh vx vy wz). Gui file nay de phan tich.
"""

import csv
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def parse(argv):
    opts = {'vmax': 0.06, 'wmax': 0.30, 'tol_mm': 5.0, 'tol_deg': 0.5, 'pose': 'ekf', 'pause': 1.0,
            'acc': 0.08, 'alpha': 0.30, 'timeout': 40.0}
    pos = []
    i = 0
    while i < len(argv):
        a = argv[i]
        if a.startswith('--'):
            k = a[2:]
            if k not in opts:
                sys.exit(f'Tuy chon la: {a}')
            opts[k] = argv[i + 1] if k == 'pose' else float(argv[i + 1])
            i += 2
        else:
            pos.append(a)
            i += 1
    if not pos:
        sys.exit(__doc__)
    cmd, args = pos[0], pos[1:]
    D = math.radians
    if cmd == 'goto':
        x, y = float(args[0]), float(args[1])
        yaw = D(float(args[2])) if len(args) > 2 else 0.0
        goals = [(x, y, yaw)]
    elif cmd == 'line':
        d = float(args[0])
        goals = [(d, 0.0, 0.0), (0.0, 0.0, 0.0)]
    elif cmd == 'turn':
        deg = float(args[0])
        n = max(1, int(math.ceil(abs(deg) / 90.0)))      # chia nho de khong cat qua ±180°
        goals = [(0.0, 0.0, D(deg * (k + 1) / n)) for k in range(n)]
    elif cmd == 'square':
        s = float(args[0])
        ccw = (args[1] if len(args) > 1 else 'ccw').lower() == 'ccw'
        sg = 1.0 if ccw else -1.0
        corners = [(s, 0.0), (s, sg * s), (0.0, sg * s), (0.0, 0.0)]
        goals, yaw = [], 0.0
        for (cx, cy) in corners:
            goals.append((cx, cy, yaw))                  # chay thang toi goc (dau xe theo canh)
            yaw += sg * math.pi / 2.0
            goals.append((cx, cy, yaw))                  # xoay 90° tai goc
    elif cmd == 'square_holo':
        s = float(args[0])
        goals = [(s, 0.0, 0.0), (s, s, 0.0), (0.0, s, 0.0), (0.0, 0.0, 0.0)]
    else:
        sys.exit(f'Lenh la: {cmd}\n' + __doc__)
    return cmd, goals, opts


class GotoTest(Node):

    def __init__(self, name, goals, o):
        super().__init__('goto_test')
        self.name, self.goals, self.o = name, goals, o
        self.vmax, self.wmax = o['vmax'], o['wmax']
        self.acc, self.alpha = o['acc'], o['alpha']
        self.tol, self.tol_yaw = o['tol_mm'] / 1000.0, math.radians(o['tol_deg'])
        self.ekf = self.odom = None
        self.start = None               # (x, y, yaw) trong khung odom, theo nguon pose
        self.yaw_unwrap = None
        self.k = 0
        self.phase = 'wait'
        self.t_phase = 0.0
        self.t_in_tol = None
        self.cmd = [0.0, 0.0, 0.0]
        self.results = []
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odometry/filtered', lambda m: self._pose(m, 'ekf'), 20)
        self.create_subscription(Odometry, '/odom', lambda m: self._pose(m, 'odom'), 20)
        path = os.path.expanduser(time.strftime('~/goto_log_%Y%m%d_%H%M%S.csv'))
        self.log_f = open(path, 'w', newline='')
        self.log = csv.writer(self.log_f)
        self.log.writerow(['t', 'dich', 'ekf_x', 'ekf_y', 'ekf_yaw_deg', 'odom_x', 'odom_y', 'odom_yaw_deg',
                           'vx', 'vy', 'wz'])
        self.log_path = path
        self.dt = 0.02
        self.t0 = self._now()
        self.create_timer(self.dt, self._tick)
        src = '/odometry/filtered (EKF)' if o['pose'] == 'ekf' else '/odom (chi banh)'
        self.get_logger().info(f'Bai "{name}": {len(goals)} dich | phan hoi: {src} | vmax {self.vmax} m/s, '
                               f'wmax {self.wmax} rad/s | dung khi < {o["tol_mm"]:.0f} mm, < {o["tol_deg"]}°')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _pose(self, m, src):
        p = m.pose.pose
        val = (p.position.x, p.position.y, yaw_of(p.orientation))
        if src == 'ekf':
            self.ekf = val
        else:
            self.odom = val

    def _fb(self):
        return self.ekf if self.o['pose'] == 'ekf' else self.odom

    def _rel(self, pose, start):
        """Pose trong khung XUAT PHAT."""
        dx, dy = pose[0] - start[0], pose[1] - start[1]
        c, s = math.cos(start[2]), math.sin(start[2])
        return (c * dx + s * dy, -s * dx + c * dy, wrap(pose[2] - start[2]))

    def _send(self, vx, vy, wz):
        # gioi han gia toc (de firmware khong phai tu cat, xe khong truot)
        lim = (self.acc * self.dt, self.acc * self.dt, self.alpha * self.dt)
        tgt = (vx, vy, wz)
        self.cmd = [c + max(-l, min(l, t - c)) for c, t, l in zip(self.cmd, tgt, lim)]
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_footprint'
        m.twist.linear.x, m.twist.linear.y, m.twist.angular.z = self.cmd
        self.pub.publish(m)

    def _tick(self):
        now = self._now()
        fb = self._fb()
        if self.phase == 'wait':
            if fb is None or (self.o['pose'] == 'ekf' and self.odom is None):
                if now - self.t0 > 5.0:
                    self.get_logger().error('Khong co /odometry/filtered hoac /odom. Chay launch use_ekf:=true chua?',
                                            throttle_duration_sec=5.0)
                return
            self.start = fb
            self.start_odom = self.odom
            self.yaw_acc = 0.0
            self.prev_yaw = fb[2]
            self.phase, self.t_phase = 'move', now
            self._announce()
            return
        if self.phase == 'done':
            return

        # yaw cong don (de xoay > 180°)
        self.yaw_acc += wrap(fb[2] - self.prev_yaw)
        self.prev_yaw = fb[2]
        rx, ry, _ = self._rel(fb, self.start)
        ryaw = self.yaw_acc
        gx, gy, gyaw = self.goals[self.k]

        if self.phase == 'pause':
            self._send(0.0, 0.0, 0.0)
            if now - self.t_phase > self.o['pause']:
                self._record()
                self.k += 1
                if self.k >= len(self.goals):
                    self._finish()
                    return
                self.phase, self.t_phase, self.t_in_tol = 'move', now, None
                self._announce()
            return

        # sai so trong khung XE
        ex, ey = gx - rx, gy - ry
        d = math.hypot(ex, ey)
        eyaw = gyaw - ryaw
        c, s = math.cos(ryaw), math.sin(ryaw)
        bx, by = c * ex + s * ey, -s * ex + c * ey

        # Bien dang toc do: v = min(vmax, sqrt(2 a d), k d). Gan dich them toc do toi thieu de thang ma sat,
        # tiep tuc tien sat dich (toi 1/3 dung sai) roi moi tha, de dung GIUA vung dung sai chu khong o mep.
        v = min(self.vmax, math.sqrt(2.0 * self.acc * 0.8 * d), 1.5 * d)
        if d > self.tol / 3.0:
            v = max(v, 0.006)
        else:
            v = 0.0
        # Huong: luon bu ti le (ke ca khi dang tinh tien), toc do toi thieu chi khi lech qua 1/3 dung sai
        w = min(self.wmax, math.sqrt(2.0 * self.alpha * 0.8 * abs(eyaw)), 2.0 * abs(eyaw))
        if abs(eyaw) > self.tol_yaw / 3.0 and d <= self.tol:
            w = max(w, 0.02)
        vx, vy = (v * bx / d, v * by / d) if d > 1e-6 else (0.0, 0.0)
        wz = math.copysign(w, eyaw)
        self._send(vx, vy, wz)
        self.log.writerow([f'{now - self.t0:.3f}', self.k + 1, *self._fmt(self.ekf), *self._fmt(self.odom),
                           *(f'{x:.4f}' for x in self.cmd)])

        if d <= self.tol and abs(eyaw) <= self.tol_yaw:
            if self.t_in_tol is None:
                self.t_in_tol = now
            elif now - self.t_in_tol > 0.3:
                self.phase, self.t_phase = 'pause', now
        else:
            self.t_in_tol = None
        if now - self.t_phase > self.o['timeout']:
            self.get_logger().error(f'Dich {self.k + 1}: qua {self.o["timeout"]:.0f} s chua toi '
                                    f'(con {d * 1000:.0f} mm, {math.degrees(eyaw):.1f}°). Bo qua dich nay')
            self.phase, self.t_phase = 'pause', now

    def _fmt(self, p):
        return (f'{p[0]:.4f}', f'{p[1]:.4f}', f'{math.degrees(p[2]):.2f}') if p else ('', '', '')

    def _announce(self):
        gx, gy, gyaw = self.goals[self.k]
        self.get_logger().info(f'→ Dich {self.k + 1}/{len(self.goals)}: x {gx * 1000:+.0f} mm, y {gy * 1000:+.0f} mm, '
                               f'yaw {math.degrees(gyaw):+.1f}°')

    def _record(self):
        gx, gy, gyaw = self.goals[self.k]
        e = self._rel(self.ekf, self.start) if self.ekf else None
        o = self._rel(self.odom, self.start_odom) if self.odom else None
        r = dict(k=self.k + 1, goal=(gx, gy, gyaw), ekf=e, odom=o, yaw_acc=self.yaw_acc)
        self.results.append(r)
        msg = (f'✓ Dich {self.k + 1}: EKF x {e[0] * 1000:+7.1f} mm  y {e[1] * 1000:+7.1f} mm  '
               f'yaw {math.degrees(self.yaw_acc):+8.2f}°')
        if o:
            msg += f' | odom x {o[0] * 1000:+7.1f}  y {o[1] * 1000:+7.1f}  yaw {math.degrees(o[2]):+7.2f}°'
        self.get_logger().info(msg)

    def _finish(self):
        self.phase = 'done'
        for _ in range(5):
            self._send(0.0, 0.0, 0.0)
        self.log_f.close()
        print('\n================ KET QUA ================')
        print(f'Bai: {self.name} | so dich: {len(self.goals)}')
        print(' dich |   muc tieu (mm, mm, do)    |     EKF (mm, mm, do)       |     odom (mm, mm, do)')
        for r in self.results:
            g, e, o = r['goal'], r['ekf'], r['odom']
            gs = f'{g[0] * 1000:+7.0f} {g[1] * 1000:+7.0f} {math.degrees(g[2]):+7.1f}'
            es = f'{e[0] * 1000:+7.1f} {e[1] * 1000:+7.1f} {math.degrees(r["yaw_acc"]):+7.2f}'
            os_ = f'{o[0] * 1000:+7.1f} {o[1] * 1000:+7.1f} {math.degrees(o[2]):+7.2f}' if o else '-'
            print(f'  {r["k"]:3d} | {gs} | {es} | {os_}')
        print('Do bang thuoc vi tri THAT cua goc truoc-trai va goc sau-trai, gui kem bang nay.')
        print(f'Log: {self.log_path}')
        print('==========================================')
        raise SystemExit(0)


def main():
    rclpy.init()
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    name, goals, opts = parse(argv)
    node = GotoTest(name, goals, opts)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        for _ in range(3):
            try:
                node._send(0.0, 0.0, 0.0)
                node.cmd = [0.0, 0.0, 0.0]
                m = TwistStamped()
                m.header.stamp = node.get_clock().now().to_msg()
                node.pub.publish(m)
            except Exception:
                pass
        if not node.log_f.closed:
            node.log_f.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
