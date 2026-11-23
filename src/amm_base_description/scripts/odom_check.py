#!/usr/bin/env python3
"""In odometry so voi DIEM XUAT PHAT, de so voi thuoc do tren xe that.

Moc = vi tri luc node nhan /odom dau tien. Muon do lai tu dau: Ctrl+C roi chay lai.

Moi 0.5 s in:
  dx, dy  : di chuyen theo truc xe LUC XUAT PHAT (mm). dx = tien, dy = sang trai
  yaw     : goc xoay CONG DON (do) theo odometry banh, khong bi cat o 180, dung cho bai xoay 5 vong
  ekf     : goc xoay cong don theo EKF (/odometry/filtered, chu yeu tu con quay IMU).
            Chi hien khi chay real_base.launch.py use_ekf:=true
  s       : quang duong da di (mm)
  vong    : so vong quay cua tung banh tu luc xuat phat, tinh tu so xung (/joint_states)

Chay:
  ros2 run amm_base_description odom_check.py
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState

WHEELS = [('FL', 'front_left_wheel_joint'), ('FR', 'front_right_wheel_joint'),
          ('RL', 'rear_left_wheel_joint'), ('RR', 'rear_right_wheel_joint')]


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class OdomCheck(Node):

    def __init__(self):
        super().__init__('odom_check')
        self.x0 = None            # (x, y, yaw) luc xuat phat
        self.prev = None          # (x, y, yaw) lan truoc
        self.yaw_sum = 0.0
        self.dist = 0.0
        self.pose = None
        self.wheel0 = None
        self.wheel = None
        self.ekf_prev = None
        self.ekf_yaw = None
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_ekf, 20)
        self.create_subscription(JointState, '/joint_states', self.on_js, 20)
        self.create_timer(0.5, self.report)
        self.get_logger().info('Cho /odom ... Moc = vi tri luc nhan goi dau tien')

    def on_odom(self, msg):
        p = msg.pose.pose
        cur = (p.position.x, p.position.y, yaw_of(p.orientation))
        if self.x0 is None:
            self.x0 = self.prev = cur
        self.yaw_sum += wrap(cur[2] - self.prev[2])
        self.dist += math.hypot(cur[0] - self.prev[0], cur[1] - self.prev[1])
        self.prev = self.pose = cur

    def on_ekf(self, msg):
        if self.x0 is None:            # cung moc voi odom: bat dau dem khi odom da co moc
            return
        yaw = yaw_of(msg.pose.pose.orientation)
        if self.ekf_prev is None:
            self.ekf_prev, self.ekf_yaw = yaw, 0.0
        self.ekf_yaw += wrap(yaw - self.ekf_prev)
        self.ekf_prev = yaw

    def on_js(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(j in pos for _, j in WHEELS):
            self.wheel = [pos[j] for _, j in WHEELS]
            if self.wheel0 is None:
                self.wheel0 = list(self.wheel)

    def report(self):
        if self.pose is None:
            return
        x0, y0, th0 = self.x0
        ddx, ddy = self.pose[0] - x0, self.pose[1] - y0
        dx = math.cos(th0) * ddx + math.sin(th0) * ddy       # quy ve truc xe luc xuat phat
        dy = -math.sin(th0) * ddx + math.cos(th0) * ddy
        line = (f'dx {dx * 1000:+8.1f} mm | dy {dy * 1000:+8.1f} mm | '
                f'yaw {math.degrees(self.yaw_sum):+8.2f} do | ')
        if self.ekf_yaw is not None:
            line += f'ekf {math.degrees(self.ekf_yaw):+8.2f} do | '
        line += f's {self.dist * 1000:7.1f} mm'
        if self.wheel is not None:
            rev = [(w - w0) / (2.0 * math.pi) for w, w0 in zip(self.wheel, self.wheel0)]
            line += ' | vong ' + ' '.join(f'{n} {r:+.3f}' for (n, _), r in zip(WHEELS, rev))
        print(line, flush=True)


def main():
    rclpy.init()
    node = OdomCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
