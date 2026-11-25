#!/usr/bin/env python3
"""GIU HUONG cho xe Mecanum bang IMU (qua EKF). Bu hien tuong xe tu xoay khi di thang.

Vi tri: amm_base_description/scripts/heading_hold_node.py

VI SAO CAN
  Do tren xe that (25/11/2026): chay thang 1 m, 4 banh quay deu (encoder thay xe thang)
  nhung con quay IMU thay xe xoay trai ~3°/m. Nguyen nhan: tai lech + banh nhua nghieng,
  encoder khong the thay. Chi IMU do duoc huong that.

LUONG
  teleop / Nav2 → /cmd_vel_raw → [node nay] → /cmd_vel → mecanum_drive_controller
                                      ↑
                     /odometry/filtered (EKF: goc yaw + toc do xoay, chu yeu tu con quay)

HAI TANG
  Tang trong (rate_loop, luon bat): bam TOC DO XOAY.
      wz_ra += k_rate * (wz_lenh - wz_do)          wz_do da loc thong thap (rate_cutoff_hz)
      Lenh "khong xoay" thi xe phai khong xoay. Dung ca khi chay Nav2.
  Tang ngoai (hold_heading): KHOA HUONG khi lenh chi co tinh tien (vx, vy) va wz = 0.
      wz_ra += k_heading * e + k_i * tich_phan(e),   e = huong_khoa - huong_do
      Thanh phan tich phan triet tieu sai lech con lai do xe xoay deu (3°/m).
      Nav2 da tu bam quy dao theo vi tri → tat tang nay khi chay Nav2 (hold_heading:=false).

KHI NAO KHONG CAN THIEP
  - Xe dung (lenh = 0): chuyen thang lenh, khong tu xoay tai cho.
  - Dang co lenh xoay (|wz| > wz_deadband): tang ngoai tat, huong khoa cap nhat theo huong moi.
    Sau khi nha phim xoay, cho settle_s roi moi khoa huong (tranh keo nguoc do qua da khi giam toc).
  - Mat EKF (> ekf_timeout_s): chuyen thang lenh, bao canh bao.
  Luong bu gioi han ±max_correction (rad/s), khong bao gio lan at lenh nguoi lai.

LOC NHIEU
  Con lan mecanum lam xe lac nhanh 2.85 Hz (±0.3°). Lac nay qua nhanh de bu
  (firmware tang toc toi da 30 RPM/s) nen loc bo truoc khi dua vao tang trong.
  Tang ngoai dung goc yaw (vot lac nho), khong loc them.

GO ROI
  /heading_hold/state  std_msgs/Float32MultiArray:
     [huong_khoa_deg, huong_do_deg, sai_lech_deg, wz_do_loc, wz_bu, dang_khoa(0/1)]

Chay (thuong qua real_base.launch.py heading_hold:=true), teleop phai phat len /cmd_vel_raw:
  ros2 run amm_base_description amm_teleop_key.py --ros-args -p use_sim:=false \\
      -p topic:=/cmd_vel_raw -p linear:=0.08 -p angular:=0.35
"""

import math

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(x, lim):
    return max(-lim, min(lim, x))


class HeadingHold(Node):

    def __init__(self):
        super().__init__('heading_hold')
        p = self.declare_parameter
        self.hold_heading = bool(p('hold_heading', True).value)
        self.rate_loop = bool(p('rate_loop', True).value)
        self.k_heading = float(p('k_heading', 1.5).value)       # (rad/s) / rad
        self.k_i = float(p('k_i', 0.5).value)                   # (rad/s) / (rad.s)
        self.k_rate = float(p('k_rate', 0.3).value)             # (rad/s) / (rad/s)
        self.max_corr = float(p('max_correction', 0.15).value)  # rad/s
        self.rate_fc = float(p('rate_cutoff_hz', 1.0).value)
        self.wz_db = float(p('wz_deadband', 0.01).value)        # rad/s: duoi muc nay coi nhu khong xoay
        self.v_db = float(p('v_deadband', 0.005).value)         # m/s: duoi muc nay coi nhu dung
        self.settle = float(p('settle_s', 0.7).value)          # ~ thoi gian firmware giam toc xoay
        self.cmd_timeout = float(p('cmd_timeout_s', 0.3).value)
        self.ekf_timeout = float(p('ekf_timeout_s', 0.3).value)
        rate_hz = float(p('rate_hz', 50.0).value)

        self.cmd = None             # (vx, vy, wz)
        self.cmd_t = None
        self.yaw = None
        self.wz_f = 0.0             # toc do xoay do, da loc
        self.ekf_t = None
        self.ekf_prev_t = None
        self.locked = False
        self.yaw_ref = 0.0
        self.integ = 0.0
        self.last_rot_t = -1e9      # luc cuoi cung co lenh xoay
        self.was_moving = False

        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.pub_state = self.create_publisher(Float32MultiArray, '~/state', 10)
        self.create_subscription(TwistStamped, '/cmd_vel_raw', self._on_cmd, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self._on_ekf, 20)
        self.dt = 1.0 / rate_hz
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f'Giu huong: tang ngoai {"BAT" if self.hold_heading else "TAT"}, tang trong '
            f'{"BAT" if self.rate_loop else "TAT"} | k_heading {self.k_heading}, k_rate {self.k_rate}, '
            f'bu toi da {self.max_corr} rad/s')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd(self, m):
        self.cmd = (m.twist.linear.x, m.twist.linear.y, m.twist.angular.z)
        self.cmd_t = self._now()

    def _on_ekf(self, m):
        t = self._now()
        self.yaw = yaw_of(m.pose.pose.orientation)
        wz = m.twist.twist.angular.z
        # Loc thong thap bac 1 (bo lac con lan 2.85 Hz truoc khi vao tang trong)
        if self.ekf_prev_t is None:
            self.wz_f = wz
        else:
            dt = max(1e-3, t - self.ekf_prev_t)
            a = dt / (dt + 1.0 / (2.0 * math.pi * self.rate_fc))
            self.wz_f += a * (wz - self.wz_f)
        self.ekf_prev_t = self.ekf_t = t

    def _publish(self, vx, vy, wz):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_footprint'
        out.twist.linear.x, out.twist.linear.y, out.twist.angular.z = vx, vy, wz
        self.pub.publish(out)

    def _tick(self):
        now = self._now()
        if self.cmd is None or now - self.cmd_t > self.cmd_timeout:
            self.locked = False          # khong co lenh: khong phat gi, controller tu dung sau reference_timeout
            return
        vx, vy, wz_cmd = self.cmd
        moving = math.hypot(vx, vy) > self.v_db
        rotating = abs(wz_cmd) > self.wz_db
        ekf_ok = self.ekf_t is not None and now - self.ekf_t < self.ekf_timeout

        if not ekf_ok:
            self.get_logger().warn('Mat /odometry/filtered: chuyen thang lenh, KHONG giu huong',
                                   throttle_duration_sec=5.0)
            self.locked = False
            self._publish(vx, vy, wz_cmd)
            return

        if rotating:
            self.last_rot_t = now
        if not moving and not rotating:
            # Xe dung: khong bu, mo khoa (lan sau bat dau di se khoa huong moi)
            self.locked = False
            self._publish(vx, vy, wz_cmd)
            self._state(0.0, 0.0)
            return

        corr = 0.0
        err = 0.0
        if self.rate_loop:
            corr += self.k_rate * (wz_cmd - self.wz_f)
        if self.hold_heading and moving and not rotating:
            if not self.locked and now - self.last_rot_t > self.settle:
                self.locked, self.yaw_ref, self.integ = True, self.yaw, 0.0
            if self.locked:
                err = wrap(self.yaw_ref - self.yaw)
                # Chong bao hoa tich phan: phan tich phan khong vuot 1/2 muc bu toi da
                lim = 0.5 * self.max_corr / max(self.k_i, 1e-6)
                self.integ = max(-lim, min(lim, self.integ + err * self.dt))
                corr += self.k_heading * err + self.k_i * self.integ
        else:
            self.locked = False
        corr = clamp(corr, self.max_corr)
        self._publish(vx, vy, wz_cmd + corr)
        self._state(err, corr)

    def _state(self, err, corr):
        m = Float32MultiArray()
        m.data = [math.degrees(self.yaw_ref) if self.locked else float('nan'),
                  math.degrees(self.yaw) if self.yaw is not None else float('nan'),
                  math.degrees(err), self.wz_f, corr, 1.0 if self.locked else 0.0]
        self.pub_state.publish(m)


def main():
    rclpy.init()
    node = HeadingHold()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
