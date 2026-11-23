#!/usr/bin/env python3
"""Driver ROS 2 cho IMU Hiwonder IM10A (giao thuc WIT, khung 0x55), cam USB thang vao Pi.

Vi tri: amm_base_description/scripts/wit_imu_node.py

PHAT
  /imu/data_raw  sensor_msgs/Imu   toc do goc (rad/s) + gia toc (m/s2) trong khung imu_link.
                                   KHONG gui huong (orientation_covariance[0] = -1, theo REP-145):
                                   EKF chi lay vyaw, huong tu module (bo loc rieng cua no) khong dung.
  /imu/euler_deg geometry_msgs/Vector3Stamped  roll, pitch, yaw (do) do CHINH module tinh (goi 0x53).
                                   Chi de doi chieu bang mat. Truc theo khung module (IMU gan up thi dau bi dao).

  Truc trong message = truc IN TREN VO module. Huong gan tren xe khai bao trong URDF (imu_joint),
  EKF tu xoay ve truc xe qua TF. KHONG doi dau trong node nay.

TAN SO
  Mac dinh module chay 9600 baud, ~10 Hz: qua cham cho EKF. Chay 1 lan:
      ros2 run amm_base_description wit_imu_config.py setup
  de dat 115200 baud, 100 Hz. Node tu do baud (thu lan luot cac muc) neu baud cau hinh khong co du lieu.

GIAO THUC WIT (moi goi 11 byte)
  0x55 | TYPE | 8 byte du lieu (4 x int16, byte thap truoc) | tong 10 byte dau (8 bit thap)
  0x51 gia toc   : x, y, z / 32768 * 16 g,      [3] nhiet do / 100 °C
  0x52 toc do goc: x, y, z / 32768 * 2000 °/s
  0x53 goc       : roll, pitch, yaw / 32768 * 180°
"""

import math
import struct
import threading
import time

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import Imu

G = 9.80665
DEG = math.pi / 180.0
TYPE_ACC, TYPE_GYRO, TYPE_ANGLE = 0x51, 0x52, 0x53
TRY_BAUDS = [115200, 9600, 230400, 460800, 921600, 57600, 38400, 19200]


class WitParser:
    """Tach goi 11 byte, kiem tra checksum."""

    def __init__(self):
        self.buf = bytearray()
        self.errors = 0

    def feed(self, data):
        self.buf += data
        out = []
        while len(self.buf) >= 11:
            if self.buf[0] != 0x55 or not 0x50 <= self.buf[1] <= 0x5F:
                del self.buf[0]
                continue
            pkt = self.buf[:11]
            if (sum(pkt[:10]) & 0xFF) != pkt[10]:
                self.errors += 1
                del self.buf[0]
                continue
            out.append((pkt[1], struct.unpack('<4h', bytes(pkt[2:10]))))
            del self.buf[:11]
        return out


class WitImuNode(Node):

    def __init__(self):
        super().__init__('wit_imu')
        p = self.declare_parameter
        self.port = p('port', '/dev/ttyUSB0').value
        self.baud = int(p('baud', 115200).value)
        self.auto_baud = bool(p('auto_baud', True).value)
        self.frame_id = p('frame_id', 'imu_link').value
        # Do lech chuan, chi la gia tri ban dau: imu_bias_calibrator do lai luc dung yen va dien covariance that
        self.gyro_std = float(p('gyro_stddev', 0.0012).value)      # rad/s (~0.07 °/s, giong mo phong)
        self.acc_std = float(p('accel_stddev', 0.01).value)        # m/s2
        self.min_rate = float(p('min_rate_warn', 50.0).value)

        self.pub_imu = self.create_publisher(Imu, '/imu/data_raw', 50)
        self.pub_euler = self.create_publisher(Vector3Stamped, '/imu/euler_deg', 10)

        self.acc = None
        self.acc_time = 0.0
        self.n = {TYPE_ACC: 0, TYPE_GYRO: 0, TYPE_ANGLE: 0}
        self.parser = WitParser()
        self.ser = None
        self.running = True
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        self.create_timer(5.0, self._report)

    # ---------------- cong noi tiep ----------------

    def _open(self, baud):
        import serial
        try:
            if self.ser is not None:
                self.ser.close()
            self.ser = serial.Serial(self.port, baud, timeout=0.05)
            self.ser.reset_input_buffer()
            return True
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'Khong mo duoc {self.port}: {e}', throttle_duration_sec=5.0)
            self.ser = None
            return False

    def _has_frames(self, baud, wait=1.2):
        """Mo o baud nay, doc trong 'wait' giay, co goi hop le khong."""
        if not self._open(baud):
            return False
        probe = WitParser()
        t_end = time.monotonic() + wait
        while time.monotonic() < t_end:
            if any(t == TYPE_GYRO for t, _ in probe.feed(self.ser.read(256))):
                return True
        return False

    def _connect(self):
        bauds = [self.baud] + ([b for b in TRY_BAUDS if b != self.baud] if self.auto_baud else [])
        while self.running:
            for b in bauds:
                if not self.running:
                    return False
                if self._has_frames(b):
                    if b != self.baud:
                        self.get_logger().warn(
                            f'IMU dang chay {b} baud (cau hinh {self.baud}). '
                            'Chay "wit_imu_config.py setup" de dat 115200 baud, 100 Hz')
                    self.get_logger().info(f'Da ket noi IMU {self.port} @ {b} baud')
                    self.parser = WitParser()
                    return True
                if self.ser is None:
                    break            # khong mo duoc cong: cho roi thu lai
            self.get_logger().error(
                f'Khong nhan duoc goi WIT tu {self.port} o cac muc baud {bauds}. Thu lai sau 2 s',
                throttle_duration_sec=10.0)
            time.sleep(2.0)
        return False

    # ---------------- doc + phat ----------------

    def _run(self):
        if not self._connect():
            return
        while self.running:
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
            except Exception as e:
                self.get_logger().error(f'Mat ket noi IMU: {e}. Dang ket noi lai')
                if not self._connect():
                    return
                continue
            if not data:
                continue
            for t, v in self.parser.feed(data):
                self._handle(t, v)

    def _handle(self, t, v):
        now = self.get_clock().now()
        with self.lock:
            if t in self.n:
                self.n[t] += 1
        if t == TYPE_ACC:
            self.acc = (v[0] / 32768.0 * 16.0 * G, v[1] / 32768.0 * 16.0 * G, v[2] / 32768.0 * 16.0 * G)
            self.acc_time = time.monotonic()
        elif t == TYPE_GYRO:
            self._publish_imu(now, [x / 32768.0 * 2000.0 * DEG for x in v[:3]])
        elif t == TYPE_ANGLE:
            e = Vector3Stamped()
            e.header.stamp = now.to_msg()
            e.header.frame_id = self.frame_id
            e.vector.x, e.vector.y, e.vector.z = (x / 32768.0 * 180.0 for x in v[:3])
            self.pub_euler.publish(e)

    def _publish_imu(self, now, w):
        m = Imu()
        m.header.stamp = now.to_msg()
        m.header.frame_id = self.frame_id
        m.orientation.w = 1.0
        m.orientation_covariance[0] = -1.0                       # khong cung cap huong
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = w
        gv = self.gyro_std ** 2
        m.angular_velocity_covariance = [gv, 0.0, 0.0, 0.0, gv, 0.0, 0.0, 0.0, gv]
        if self.acc is not None and time.monotonic() - self.acc_time < 0.1:
            m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = self.acc
            av = self.acc_std ** 2
            m.linear_acceleration_covariance = [av, 0.0, 0.0, 0.0, av, 0.0, 0.0, 0.0, av]
        else:
            m.linear_acceleration_covariance[0] = -1.0           # module khong gui goi gia toc
        self.pub_imu.publish(m)

    def _report(self):
        with self.lock:
            n = dict(self.n)
            for k in self.n:
                self.n[k] = 0
        hz = {k: v / 5.0 for k, v in n.items()}
        if self.ser is None or hz[TYPE_GYRO] == 0:
            self.get_logger().warn(f'Chua co du lieu con quay tu {self.port}')
            return
        msg = (f'IMU: gyro {hz[TYPE_GYRO]:.0f} Hz | acc {hz[TYPE_ACC]:.0f} Hz | '
               f'goc {hz[TYPE_ANGLE]:.0f} Hz | loi checksum {self.parser.errors}')
        if hz[TYPE_GYRO] < self.min_rate:
            self.get_logger().warn(msg + f' | < {self.min_rate:.0f} Hz: chay "wit_imu_config.py setup"')
        else:
            self.get_logger().info(msg)

    def shutdown(self):
        self.running = False
        self.thread.join(timeout=1.0)
        if self.ser is not None:
            self.ser.close()


def main():
    rclpy.init()
    node = WitImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
