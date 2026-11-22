#!/usr/bin/env python3
"""So cac nguon uoc luong chuyen dong voi vi tri that trong Gazebo.

  /odom              : odometry banh (mecanum_drive_controller)
  /imu               : con quay IMU THO (con bias), node nay tu cong don theta += wz * dt
  /odometry/filtered : ket qua EKF (chi co khi use_ekf:=true)
  /ground_truth/odom : vi tri that (chi co trong mo phong)

Moi giay in 2 dong:
  - quang duong da di, sai so vi tri cua odometry banh va EKF
  - goc yaw cua tung nguon va do lech so voi goc that

Goc IMU duoc dat bang goc that o lan doc dau tien, de cac nguon cung moc.
Muon so tu dau: chay node nay truoc khi lai xe.
Muon xem con quay DA tru bias: -p imu_topic:=/imu/corrected

Chay (che do drive_mode:=control):
  ros2 run amm_base_description odom_error_monitor.py
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def yaw_of(q):
    """Goc yaw (rad) tu quaternion, xem bai 016-018 thu muc TF2."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def stamp_sec(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class PoseTrack:
    """Vi tri moi nhat va goc yaw cong don (khong nhay 360° khi qua ±180°)."""

    def __init__(self):
        self.xy = None
        self.yaw_total = None

    def update(self, msg):
        p = msg.pose.pose
        self.xy = (p.position.x, p.position.y)
        yaw = yaw_of(p.orientation)
        if self.yaw_total is None:
            self.yaw_total = yaw
        else:
            self.yaw_total += wrap(yaw - wrap(self.yaw_total))


class OdomErrorMonitor(Node):
    def __init__(self):
        super().__init__('odom_error_monitor')
        imu_topic = self.declare_parameter('imu_topic', '/imu').value
        self.imu_label = 'IMU' if imu_topic == '/imu' else 'IMUsua'
        self.odom = PoseTrack()
        self.ekf = PoseTrack()
        self.truth = PoseTrack()
        self.last_truth_xy = None
        self.distance = 0.0
        self.imu_yaw = None
        self.imu_last_t = None
        self.create_subscription(Odometry, '/odom', self.odom.update, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.ekf.update, 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_truth, 10)
        self.create_subscription(Imu, imu_topic, self.on_imu, qos_profile_sensor_data)
        self.create_timer(1.0, self.report)

    def on_truth(self, msg):
        self.truth.update(msg)
        if self.last_truth_xy is not None:
            self.distance += math.dist(self.truth.xy, self.last_truth_xy)
        self.last_truth_xy = self.truth.xy

    def on_imu(self, msg):
        t = stamp_sec(msg)
        if self.imu_yaw is None:
            if self.truth.yaw_total is None:
                return  # cho goc that de dat moc
            self.imu_yaw = self.truth.yaw_total
            self.imu_last_t = t
            return
        dt = t - self.imu_last_t
        self.imu_last_t = t
        if 0.0 < dt < 0.5:  # bo qua khi Gazebo tam dung hoac reset
            self.imu_yaw += msg.angular_velocity.z * dt

    def pos_err(self, track):
        if track.xy is None:
            return '   -  '
        e = math.dist(track.xy, self.truth.xy) * 1000
        return f'{e:6.1f}'

    def yaw_str(self, label, yaw_total, t_deg):
        if yaw_total is None:
            return f'{label}    -'
        d = math.degrees(yaw_total)
        return f'{label} {d:+7.1f}° ({d - t_deg:+5.1f})'

    def report(self):
        missing = [n for n, v in (('/odom', self.odom.xy), ('/ground_truth/odom', self.truth.xy))
                   if v is None]
        if missing:
            self.get_logger().info(f'Dang cho {", ".join(missing)} ...')
            return
        t_deg = math.degrees(self.truth.yaw_total)
        self.get_logger().info(
            f'di {self.distance:5.2f} m | lech vi tri (mm): odom {self.pos_err(self.odom)}'
            f'  EKF {self.pos_err(self.ekf)}\n'
            f'  yaw that {t_deg:+7.1f}° | '
            + ' | '.join([self.yaw_str('odom', self.odom.yaw_total, t_deg),
                          self.yaw_str(self.imu_label, self.imu_yaw, t_deg),
                          self.yaw_str('EKF', self.ekf.yaw_total, t_deg)]))


def main():
    rclpy.init()
    node = OdomErrorMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
