#!/usr/bin/env python3
"""Hieu chuan bias con quay luc khoi dong va dien covariance cho IMU.

Dung chung cho mo phong (Gazebo) va xe that (Hiwonder IM10A, driver wit_ros2_imu).

  /imu (hoac /imu/data_raw tren xe that)  →  [node nay]  →  /imu/corrected  →  EKF

Cach lam:
  1. Hieu chuan: trong calib_duration giay dau, khi xe DUNG YEN (encoder bao van toc ≈ 0),
     gom mau van toc goc. Van toc that = 0 nen trung binh mau = bias, do lech chuan = nhieu.
     Neu xe chuyen dong giua chung, bo mau va lam lai.
     Trong luc hieu chuan KHONG phat /imu/corrected (EKF tam chi dung encoder).
  2. Sau do, moi mau:
       angular_velocity  = do − bias            (neu enable_bias_correction)
       covariance van toc goc = max(cov_scale · σ_do, cov_floor)²   tren duong cheo
       frame_id = frame_id (mac dinh imu_link, khop URDF)
     Gia toc: giu nguyen gia tri. Covariance bang 0 thi dien theo σ do duoc (cung quy tac).
     Huong (orientation): giu nguyen. Neu message khong co huong (quaternion toan 0)
     thi dat orientation_covariance[0] = -1 theo REP-145 ("khong cung cap").

Vi sao khong dung nguyen σ do duoc: 3 giay dung yen chi do duoc nhieu trang.
Khi chay con rung dong co, bias sot, sai so ti le. σ qua nho thi EKF bo qua han encoder.

Tham so:
  input_topic             (/imu)
  output_topic            (/imu/corrected)
  odom_topic              (/odom)   dung de kiem tra xe dung yen
  frame_id                (imu_link) de trong "" thi giu frame cua driver
  calib_duration          (3.0)  s
  enable_bias_correction  (true)  false: chi dien covariance + doi frame_id, khong tru bias
  cov_scale               (3.0)  k
  cov_floor               (0.002) rad/s, sigma toi thieu cua con quay
  accel_cov_floor         (0.02) m/s², sigma toi thieu cua gia toc ke
  still_linear            (0.005) m/s,  nguong coi la dung yen
  still_angular           (0.005) rad/s
  require_odom            (true)  false: khong co /odom thi van hieu chuan (gia dinh dung yen)
  use_sim                 (true)  dung dong ho /clock. Xe that: false

Chay rieng:
  ros2 run amm_base_description imu_bias_calibrator.py
Xe that (vi du):
  ros2 run amm_base_description imu_bias_calibrator.py --ros-args \\
    -p input_topic:=/imu/data_raw -p use_sim:=false
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def stamp_sec(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class RunningStats:
    """Trung binh va do lech chuan cho 3 truc (thuat toan Welford)."""

    def __init__(self):
        self.n = 0
        self.mean = [0.0, 0.0, 0.0]
        self.m2 = [0.0, 0.0, 0.0]

    def add(self, v):
        self.n += 1
        for i in range(3):
            d = v[i] - self.mean[i]
            self.mean[i] += d / self.n
            self.m2[i] += d * (v[i] - self.mean[i])

    def std(self):
        if self.n < 2:
            return [0.0, 0.0, 0.0]
        return [math.sqrt(m / (self.n - 1)) for m in self.m2]


class ImuBiasCalibrator(Node):
    def __init__(self):
        super().__init__('imu_bias_calibrator')
        p = self.declare_parameter
        in_topic = p('input_topic', '/imu').value
        out_topic = p('output_topic', '/imu/corrected').value
        odom_topic = p('odom_topic', '/odom').value
        self.frame_id = p('frame_id', 'imu_link').value
        self.calib_duration = float(p('calib_duration', 3.0).value)
        self.enable_bias = bool(p('enable_bias_correction', True).value)
        self.cov_scale = float(p('cov_scale', 3.0).value)
        self.cov_floor = float(p('cov_floor', 0.002).value)
        self.accel_cov_floor = float(p('accel_cov_floor', 0.02).value)
        self.still_lin = float(p('still_linear', 0.005).value)
        self.still_ang = float(p('still_angular', 0.005).value)
        self.require_odom = bool(p('require_odom', True).value)
        if p('use_sim', True).value:
            self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.moving = None          # None = chua co /odom
        self.gyro = RunningStats()
        self.accel = RunningStats()
        self.calib_start = None
        self.calibrated = False
        self.bias = [0.0, 0.0, 0.0]
        self.gyro_var = [0.0, 0.0, 0.0]
        self.accel_var = [0.0, 0.0, 0.0]
        self.wait_logged = False

        self.pub = self.create_publisher(Imu, out_topic, 10)
        self.create_subscription(Imu, in_topic, self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 10)
        self.get_logger().info(
            f'Doc {in_topic}, phat {out_topic}. Giu xe DUNG YEN {self.calib_duration:.1f} s de hieu chuan...')

    def on_odom(self, msg):
        t = msg.twist.twist
        self.moving = (math.hypot(t.linear.x, t.linear.y) > self.still_lin
                       or abs(t.angular.z) > self.still_ang)

    def reset_calibration(self, reason):
        if self.gyro.n:
            self.get_logger().warn(f'{reason}: bo {self.gyro.n} mau, hieu chuan lai')
        self.gyro = RunningStats()
        self.accel = RunningStats()
        self.calib_start = None

    def calibrate(self, msg):
        if self.moving is None and self.require_odom:
            if not self.wait_logged:
                self.get_logger().info('Dang cho /odom de kiem tra xe dung yen...')
                self.wait_logged = True
            return
        if self.moving:
            self.reset_calibration('Xe dang chuyen dong')
            return

        t = stamp_sec(msg)
        if self.calib_start is not None and t < self.calib_start:
            self.reset_calibration('Dong ho chay lui (mo phong bi reset)')
        if self.calib_start is None:
            self.calib_start = t
        w, a = msg.angular_velocity, msg.linear_acceleration
        self.gyro.add((w.x, w.y, w.z))
        self.accel.add((a.x, a.y, a.z))

        if t - self.calib_start < self.calib_duration or self.gyro.n < 20:
            return

        g_std, a_std = self.gyro.std(), self.accel.std()
        self.bias = list(self.gyro.mean) if self.enable_bias else [0.0, 0.0, 0.0]
        self.gyro_var = [max(self.cov_scale * s, self.cov_floor) ** 2 for s in g_std]
        self.accel_var = [max(self.cov_scale * s, self.accel_cov_floor) ** 2 for s in a_std]
        self.calibrated = True

        deg = math.degrees
        self.get_logger().info(
            f'Hieu chuan xong voi {self.gyro.n} mau.\n'
            f'  bias con quay z = {deg(self.gyro.mean[2]):+.4f} °/s '
            f'({deg(self.gyro.mean[2]) * 60:+.2f} °/phut neu khong tru)'
            f'{"" if self.enable_bias else "  [KHONG tru, enable_bias_correction=false]"}\n'
            f'  sigma do duoc z = {g_std[2]:.5f} rad/s, sigma dua vao EKF = {math.sqrt(self.gyro_var[2]):.5f} rad/s '
            f'(k = {self.cov_scale}, san = {self.cov_floor})\n'
            f'  bias con sot (uoc luong) = {deg(g_std[2] / math.sqrt(self.gyro.n)) * 60:.2f} °/phut')

    def on_imu(self, msg):
        if not self.calibrated:
            self.calibrate(msg)
            return

        out = Imu()
        out.header = msg.header
        if self.frame_id:
            out.header.frame_id = self.frame_id

        w = msg.angular_velocity
        out.angular_velocity.x = w.x - self.bias[0]
        out.angular_velocity.y = w.y - self.bias[1]
        out.angular_velocity.z = w.z - self.bias[2]
        out.angular_velocity_covariance = [0.0] * 9
        for i in range(3):
            out.angular_velocity_covariance[4 * i] = self.gyro_var[i]

        out.linear_acceleration = msg.linear_acceleration
        acc_cov = list(msg.linear_acceleration_covariance)
        if not any(acc_cov):
            acc_cov = [0.0] * 9
            for i in range(3):
                acc_cov[4 * i] = self.accel_var[i]
        out.linear_acceleration_covariance = acc_cov

        q = msg.orientation
        out.orientation = q
        if q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0:
            out.orientation.w = 1.0
            ori_cov = [0.0] * 9
            ori_cov[0] = -1.0   # REP-145: khong cung cap huong
        else:
            ori_cov = list(msg.orientation_covariance)
        out.orientation_covariance = ori_cov

        self.pub.publish(out)


def main():
    rclpy.init()
    node = ImuBiasCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
