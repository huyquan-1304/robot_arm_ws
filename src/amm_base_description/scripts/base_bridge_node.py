#!/usr/bin/env python3
"""Cau noi ROS 2 <-> STM32 cho XE 4 BANH (firmware base_link.c), chi phan xe, chua co IMU.

Vi tri: amm_base_description/scripts/base_bridge_node.py

LUONG DU LIEU (giong mo phong, STM32 thay cho Gazebo):
  /cmd_vel -> mecanum_drive_controller -> JointStateTopicSystem
      --/base/wheel_commands (JointState.velocity, rad/s)--> node nay --0x10 CMD_BASE_VEL--> STM32
  STM32 --0x90 STATE_BASE (so xung, toc do do)--> node nay
      --/base/wheel_states (position rad, velocity rad/s)--> JointStateTopicSystem
      -> mecanum_drive_controller tinh /odom, joint_state_broadcaster phat /joint_states

DOI DON VI (firmware khong biet URDF):
  goc banh (rad)  = wheel_sign * so_xung * 2*pi / counts_per_rev
  toc do (rad/s)  = wheel_sign * toc_do_STM32 (mrad/s) / 1000
  lenh (mrad/s)   = wheel_sign * lenh (rad/s) * 1000
  Toc do lay tu STM32 (do bang encoder, cua so 20 ms, dinh thoi bang ngat TIM6) vi
  mecanum_drive_controller tinh odometry bang cach cong don TOC DO banh, khong dung goc banh.

HAI CHE DO
  loopback:=false  Cong noi tiep that (USB CDC /dev/ttyACM0 hoac USART1 qua USB-UART).
  loopback:=true   Gia lap STM32 ngay trong node (gioi han 25 RPM, tang toc 30 RPM/s, watchdog),
                   khong can phan cung. Dung de thu toan bo phan ROS tren may ao truoc.

AN TOAN
  - Mac dinh TAT. Goi ~/enable de xe nhan lenh (hoac auto_enable:=true khi thu loopback).
  - Khi TAT van gui PING moi chu ky: STM32 o lai che do ROS va van gui so xung (quay tay de kiem tra).
  - Lenh tu controller cu qua cmd_timeout_s: gui toc do 0.
  - STM32 bao ket truc / nguoc encoder (FAULT), ESTOP, hoac tu tat: node TAT va bao loi.
    Mot banh bi ngat ma 3 banh con lai van chay thi xe quay lech, nen tat ca 4.
  - Mat STATE qua state_timeout_s: TAT.
  - Dung node (Ctrl+C): gui lenh TAT.

DICH VU
  ros2 service call /base_bridge/enable std_srvs/srv/SetBool "{data: true}"
  ros2 service call /base_bridge/enable std_srvs/srv/SetBool "{data: false}"
  ros2 service call /base_bridge/estop std_srvs/srv/Trigger
"""

import math
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt16
from std_srvs.srv import SetBool, Trigger

# ----------------------------------------------------------------------
# GIAO THUC: PHAI TRUNG VOI base_link.h / ros_link.h
# ----------------------------------------------------------------------
HEADER = b'\xAA\x55'

T_CMD_ESTOP = 0x03          # chung voi tay may
T_CMD_BASE_VEL = 0x10       # seq u16 | 4 x i16 (mrad/s)                                  10 byte
T_CMD_BASE_ENABLE = 0x11    # on u8
T_CMD_BASE_PING = 0x12      # rong
T_STATE_BASE = 0x90         # seq u16 | counts 4 x i32 | w 4 x i16 (mrad/s) | status u16  28 byte

FMT_CMD_BASE_VEL = '<H4h'
FMT_STATE_BASE = '<H4i4hH'
LEN_STATE_BASE = struct.calcsize(FMT_STATE_BASE)

ST_ENABLED = 1 << 0
ST_WATCHDOG = 1 << 1
ST_FAULT_MASK = 0b1111 << 2     # FAULT_FL, FAULT_FR, FAULT_RL, FAULT_RR
ST_SATURATED = 1 << 6
ST_I2C_ERROR = 1 << 7
ST_ESTOP = 1 << 8
STATUS_BITS = {
    0: 'ENABLED', 1: 'WATCHDOG', 2: 'FAULT_FL', 3: 'FAULT_FR', 4: 'FAULT_RL',
    5: 'FAULT_RR', 6: 'SATURATED', 7: 'I2C_ERROR', 8: 'ESTOP',
}
# Bit "ben vung" de in khi thay doi. WATCHDOG, SATURATED, I2C_ERROR nhap nhay nen bao rieng (co gioi han tan suat)
STATUS_LOG_MASK = ST_ENABLED | ST_FAULT_MASK | ST_ESTOP

WHEEL_TAGS = ['FL', 'FR', 'RL', 'RR']   # thu tu banh trong khung: 0 FL, 1 FR, 2 RL, 3 RR


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_frame(msg_type: int, payload: bytes = b'') -> bytes:
    body = bytes([msg_type, len(payload)]) + payload
    return HEADER + body + struct.pack('<H', crc16_ccitt(body))


class FrameParser:
    """Tach khung tu dong byte. Bo qua chu console STM32 in ra truoc khi vao che do ROS."""

    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = 0

    def feed(self, data: bytes):
        self.buf += data
        frames = []
        while True:
            start = self.buf.find(HEADER)
            if start < 0:
                del self.buf[:-1]
                return frames
            del self.buf[:start]
            if len(self.buf) < 4:
                return frames
            length = self.buf[3]
            if length > 64:                     # khong co goi nao dai vay: header gia
                del self.buf[:1]
                continue
            total = 6 + length
            if len(self.buf) < total:
                return frames
            body = bytes(self.buf[2:4 + length])
            crc_rx = struct.unpack('<H', self.buf[4 + length:total])[0]
            if crc16_ccitt(body) == crc_rx:
                frames.append((body[0], body[2:]))
                del self.buf[:total]
            else:
                self.crc_errors += 1
                del self.buf[:1]


def clamp_i16(x: float) -> int:
    return max(-32768, min(32767, int(round(x))))


# ----------------------------------------------------------------------
# GIA LAP STM32 (loopback): chep logic base_link.c, banh bam lenh ngay (khong mo phong PID)
# ----------------------------------------------------------------------
class LoopbackBase:
    RPM_LIMIT = 25.0
    ACC_RPM_S = 30.0
    WATCHDOG_S = 0.2

    def __init__(self, counts_per_rev):
        self.cpr = counts_per_rev
        self.enabled = False
        self.estop = False
        self.wd_trip = False
        self.saturated = False
        self.seq = 0
        self.tgt = [0.0] * 4      # RPM
        self.cur = [0.0] * 4      # RPM
        self.pos = [0.0] * 4      # so xung (so thuc)
        self.since_cmd = 0.0

    def rx(self, msg_type, payload):
        if msg_type == T_CMD_BASE_ENABLE and len(payload) == 1:
            if payload[0]:
                self.enabled, self.estop, self.wd_trip, self.saturated = True, False, False, False
                self.tgt, self.cur, self.since_cmd = [0.0] * 4, [0.0] * 4, 0.0
            else:
                self.enabled, self.tgt, self.cur = False, [0.0] * 4, [0.0] * 4
        elif msg_type == T_CMD_ESTOP:
            self.enabled, self.estop, self.tgt, self.cur = False, True, [0.0] * 4, [0.0] * 4
        elif msg_type == T_CMD_BASE_VEL and len(payload) == 10:
            v = struct.unpack(FMT_CMD_BASE_VEL, payload)
            self.seq = v[0]
            if not self.enabled:
                return
            rpm = [w / 1000.0 * 60.0 / (2.0 * math.pi) for w in v[1:5]]
            rmax = max(abs(r) for r in rpm)
            k = self.RPM_LIMIT / rmax if rmax > self.RPM_LIMIT else 1.0
            self.saturated = k < 1.0
            self.tgt = [r * k for r in rpm]
            self.since_cmd, self.wd_trip = 0.0, False

    def step(self, dt):
        if self.enabled:
            self.since_cmd += dt
            if not self.wd_trip and self.since_cmd > self.WATCHDOG_S:
                self.tgt, self.wd_trip, self.saturated = [0.0] * 4, True, False
            d = [t - c for t, c in zip(self.tgt, self.cur)]
            dmax = max(abs(x) for x in d)
            step = self.ACC_RPM_S * dt
            k = step / dmax if dmax > step else 1.0
            self.cur = [c + x * k for c, x in zip(self.cur, d)]
        for i in range(4):
            self.pos[i] += self.cur[i] / 60.0 * self.cpr * dt
        st = ((ST_ENABLED if self.enabled else 0) | (ST_WATCHDOG if self.wd_trip else 0)
              | (ST_SATURATED if self.saturated else 0) | (ST_ESTOP if self.estop else 0))
        w = [clamp_i16(c * 2.0 * math.pi / 60.0 * 1000.0) for c in self.cur]
        counts = [int(round(p)) for p in self.pos]
        return struct.pack(FMT_STATE_BASE, self.seq, *counts, *w, st)


# ----------------------------------------------------------------------
# NODE
# ----------------------------------------------------------------------
class BaseBridge(Node):

    def __init__(self):
        super().__init__('base_bridge')
        p = self.declare_parameter
        p('port', '/dev/ttyACM0')
        p('baud', 115200)                  # USB CDC bo qua baud. USART1 qua USB-UART: 115200
        p('rate_hz', 50.0)                 # gui lenh 50 Hz, STM32 gui STATE_BASE 50 Hz
        p('loopback', False)
        p('auto_enable', False)            # chi dung khi thu loopback. Xe that: goi ~/enable bang tay
        p('joints', ['front_left_wheel_joint', 'front_right_wheel_joint',
                     'rear_left_wheel_joint', 'rear_right_wheel_joint'])   # thu tu FL, FR, RL, RR
        p('wheel_sign', [1.0, 1.0, 1.0, 1.0])
        p('counts_per_rev', 7392)          # 11 xung x 4 canh x hop so 168 (BASE_COUNTS_PER_REV)
        p('max_wheel_speed', 2.618)        # rad/s = 25 RPM (BASE_RPM_LIMIT). Chi de canh bao
        p('cmd_timeout_s', 0.3)
        p('state_timeout_s', 0.5)
        p('commands_topic', '/base/wheel_commands')
        p('states_topic', '/base/wheel_states')

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.port, self.baud, self.rate = g('port'), int(g('baud')), float(g('rate_hz'))
        self.loopback = bool(g('loopback'))
        self.joints = list(g('joints'))
        self.sign = [float(v) for v in g('wheel_sign')]
        self.cpr = int(g('counts_per_rev'))
        self.max_w = float(g('max_wheel_speed'))
        self.cmd_timeout = float(g('cmd_timeout_s'))
        self.state_timeout = float(g('state_timeout_s'))
        if len(self.joints) != 4 or len(self.sign) != 4:
            raise ValueError('joints va wheel_sign phai co dung 4 phan tu, thu tu FL, FR, RL, RR')
        self.rad_per_count = 2.0 * math.pi / self.cpr

        self.cmd = [0.0] * 4             # lenh moi nhat tu controller, rad/s
        self.cmd_time = None
        self.counts = None               # so xung moi nhat
        self.w_meas = [0.0] * 4          # toc do do, rad/s
        self.status = 0
        self.enabled = False
        self.seq = 0
        self.last_state_time = None
        self.enable_grace_until = 0
        self.auto_enable_pending = bool(g('auto_enable'))
        self.parser = FrameParser()
        self.serial = None
        self.next_open_try = 0
        self.n_state = 0
        self.lb = LoopbackBase(self.cpr) if self.loopback else None
        self.lb_last_ns = 0

        if self.loopback:
            self.get_logger().warn('CHE DO LOOPBACK: gia lap STM32, KHONG dieu khien xe that')
        else:
            self._open_serial()

        self.pub_state = self.create_publisher(JointState, g('states_topic'), 10)
        self.pub_status = self.create_publisher(UInt16, '~/status', 10)
        self.create_subscription(JointState, g('commands_topic'), self._on_cmd, 10)
        self.create_service(SetBool, '~/enable', self._srv_enable)
        self.create_service(Trigger, '~/estop', self._srv_estop)
        self.create_timer(1.0 / self.rate, self._tick)
        self.create_timer(5.0, self._report)

    # ---------------- cong noi tiep ----------------

    def _open_serial(self):
        import serial  # python3-serial
        try:
            self.serial = serial.Serial(self.port, baudrate=self.baud, timeout=0)
            self.serial.reset_input_buffer()
            self.get_logger().info(f'Da mo {self.port}')
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'Khong mo duoc {self.port}: {e}. Thu lai moi 1 s',
                                    throttle_duration_sec=5.0)
            self.serial = None

    def _send(self, msg_type, payload=b''):
        if self.loopback:
            self.lb.rx(msg_type, payload)
            return
        if self.serial is None:
            return
        try:
            self.serial.write(build_frame(msg_type, payload))
        except Exception as e:  # rut cap USB
            self.get_logger().error(f'Loi ghi cong noi tiep: {e}')
            self._close_serial()

    def _read_frames(self):
        if self.loopback:
            now_ns = self.get_clock().now().nanoseconds
            dt = min(0.1, (now_ns - self.lb_last_ns) * 1e-9) if self.lb_last_ns else 1.0 / self.rate
            self.lb_last_ns = now_ns
            return [(T_STATE_BASE, self.lb.step(dt))]
        if self.serial is None:
            return []
        try:
            data = self.serial.read(self.serial.in_waiting or 0)
        except Exception as e:
            self.get_logger().error(f'Loi doc cong noi tiep: {e}')
            self._close_serial()
            return []
        return self.parser.feed(data) if data else []

    def _close_serial(self):
        try:
            self.serial.close()
        except Exception:
            pass
        self.serial = None
        if self.enabled:
            self.enabled = False
            self.get_logger().error('Mat ket noi STM32, TAT. Cam lai roi goi ~/enable')

    # ---------------- callback ----------------

    def _on_cmd(self, msg: JointState):
        if not msg.velocity:
            return
        vel = dict(zip(msg.name, msg.velocity))
        for i, j in enumerate(self.joints):
            if j in vel:
                v = vel[j]
                self.cmd[i] = v if math.isfinite(v) else 0.0   # NaN: controller chua chay
        self.cmd_time = self.get_clock().now()

    def _srv_enable(self, req, res):
        if req.data and self.counts is None:
            res.success, res.message = False, 'Chua nhan STATE_BASE tu STM32, khong bat'
            return res
        if req.data and self.status & ST_FAULT_MASK:
            self.get_logger().warn('STM32 dang bao FAULT, lenh BAT se xoa loi. Kiem tra banh truoc!')
        self.cmd = [0.0] * 4
        self._send(T_CMD_BASE_ENABLE, b'\x01' if req.data else b'\x00')
        self.enabled = bool(req.data)
        self.enable_grace_until = self.get_clock().now().nanoseconds + int(0.3e9)
        res.success, res.message = True, 'BAT' if req.data else 'TAT'
        self.get_logger().info(f'Xe: {res.message}')
        return res

    def _srv_estop(self, req, res):
        self.enabled = False
        self._send(T_CMD_ESTOP)
        self.get_logger().error('DUNG KHAN CAP. Goi ~/enable de chay lai')
        res.success, res.message = True, 'ESTOP'
        return res

    def _try_auto_enable(self):
        if self.auto_enable_pending and self.counts is not None:
            self.auto_enable_pending = False
            self._srv_enable(SetBool.Request(data=True), SetBool.Response())

    # ---------------- vong lap chinh ----------------

    def _tick(self):
        now = self.get_clock().now()

        # 0. Chua mo duoc cong: thu lai moi 1 s
        if not self.loopback and self.serial is None and now.nanoseconds >= self.next_open_try:
            self.next_open_try = now.nanoseconds + int(1e9)
            self._open_serial()
            self.parser = FrameParser()

        # 1. Nhan trang thai, chi dung goi moi nhat
        latest = None
        for msg_type, payload in self._read_frames():
            if msg_type == T_STATE_BASE and len(payload) == LEN_STATE_BASE:
                latest = struct.unpack(FMT_STATE_BASE, payload)
                self.n_state += 1
        if latest is not None:
            self.counts = latest[1:5]
            self.w_meas = [s * w / 1000.0 for s, w in zip(self.sign, latest[5:9])]
            self.last_state_time = now
            self._update_status(latest[9])
            self._publish_state(now)

        self._try_auto_enable()

        # 2. Mat trang thai qua lau: TAT, va bao toc do 0 de odometry khong tiep tuc cong don toc do cu
        if latest is None and self.last_state_time is not None:
            age = (now - self.last_state_time).nanoseconds * 1e-9
            if age > self.state_timeout:
                if self.enabled:
                    self.enabled = False
                    self.get_logger().error(f'Mat STATE_BASE {age:.2f} s, TAT. Kiem tra cap USB / STM32')
                if any(self.w_meas):
                    self.w_meas = [0.0] * 4
                    self._publish_state(now)

        # 3. Gui lenh toc do, hoac PING de STM32 o lai che do ROS
        if not self.enabled:
            self._send(T_CMD_BASE_PING)
            if any(abs(c) > 1e-3 for c in self.cmd) and self.cmd_time is not None:
                self.get_logger().warn('Co lenh chay nhung xe dang TAT. Goi ~/enable',
                                       throttle_duration_sec=5.0)
            return
        cmd = self.cmd
        if self.cmd_time is None or (now - self.cmd_time).nanoseconds * 1e-9 > self.cmd_timeout:
            cmd = [0.0] * 4
        peak = max(abs(c) for c in cmd)
        if peak > self.max_w * 1.001:
            self.get_logger().warn(
                f'Lenh banh {peak:.2f} rad/s > {self.max_w:.2f} rad/s (25 RPM). STM32 se thu nho deu '
                '4 banh. Giam gioi han toc do o teleop / controller.', throttle_duration_sec=5.0)
        self.seq = (self.seq + 1) & 0xFFFF
        mrad = [clamp_i16(s * c * 1000.0) for s, c in zip(self.sign, cmd)]
        self._send(T_CMD_BASE_VEL, struct.pack(FMT_CMD_BASE_VEL, self.seq, *mrad))

    def _publish_state(self, now):
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = list(self.joints)
        js.position = [s * c * self.rad_per_count for s, c in zip(self.sign, self.counts)]
        js.velocity = list(self.w_meas)
        self.pub_state.publish(js)

    def _update_status(self, status):
        changed = (status ^ self.status) & STATUS_LOG_MASK
        if changed:
            names = [n for b, n in STATUS_BITS.items() if status & (1 << b) & STATUS_LOG_MASK]
            msg = f'Trang thai STM32: {names or ["TAT"]}'
            if status & (ST_FAULT_MASK | ST_ESTOP):
                self.get_logger().error(msg)
            else:
                self.get_logger().info(msg)
        self.status = status
        self.pub_status.publish(UInt16(data=status))

        if status & ST_I2C_ERROR:
            self.get_logger().warn('STM32 bao loi I2C toi module MKE-M18', throttle_duration_sec=2.0)
        if self.enabled and status & ST_SATURATED:
            self.get_logger().warn('STM32 bao SATURATED: lenh vuot 25 RPM, da thu nho deu',
                                   throttle_duration_sec=5.0)
        if self.enabled and status & ST_WATCHDOG:
            self.get_logger().warn('STM32 bao WATCHDOG: lenh toc do toi khong deu (> 200 ms)',
                                   throttle_duration_sec=2.0)

        # Vua BAT: STATE dau tien co the gui truoc khi STM32 xu ly lenh BAT (con bit FAULT / ESTOP cu)
        if not self.enabled or self.get_clock().now().nanoseconds < self.enable_grace_until:
            return
        if status & ST_FAULT_MASK:
            bad = [WHEEL_TAGS[i] for i in range(4) if status & (1 << (2 + i))]
            self.enabled = False
            self._send(T_CMD_BASE_ENABLE, b'\x00')
            self.get_logger().error(
                f'Banh {bad} bi ngat (ket truc hoac encoder nguoc chieu). TAT ca 4 banh. '
                'Kiem tra roi goi ~/enable')
        elif status & ST_ESTOP:
            self.enabled = False
        elif not status & ST_ENABLED:
            self.enabled = False
            self.get_logger().error('STM32 da tu TAT (thoat che do ROS?). Goi lai ~/enable')

    def _report(self):
        """Moi 5 s: tan so STATE va loi CRC, de biet duong truyen co on khong."""
        if self.last_state_time is None:
            src = 'loopback' if self.loopback else self.port
            self.get_logger().warn(f'Chua nhan STATE_BASE tu {src}. Firmware moi da nap chua? Dung cong chua?')
            return
        hz = self.n_state / 5.0
        self.n_state = 0
        level = self.get_logger().info if hz > 40.0 else self.get_logger().warn
        level(f'STATE {hz:.0f} Hz | loi CRC {self.parser.crc_errors} | '
              f'{"BAT" if self.enabled else "TAT"}')

    def shutdown(self):
        for _ in range(3):
            self._send(T_CMD_BASE_ENABLE, b'\x00')


def main():
    rclpy.init()
    node = BaseBridge()
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
