#!/usr/bin/env python3
"""Gia lap STM32 + 4 driver ZDT X42S + servo kep, noi DUNG giao thuc nhi phan cua ros_link.c.

Vi tri: robot_arm_4_dof/node/stm32_emulator_node.py

Muc dich: chay THU TOAN BO phan mem phia Pi (node cau noi, ros2_control, node gap) truoc khi nap
firmware va cap 24 V. Node nay tao mot cong noi tiep ao (vi du /tmp/ttySTM32). stm32_bridge_node
mo cong nay y nhu mo /dev/ttyUSB0 that, nen moi byte, moi phep doi don vi deu duoc chay that.

Bat chuoc firmware (ros_link.c + joint.c):
  - Bang khop: ti so truyen, chieu, gioi han goc, toc do va gia toc toi da (config/stm32_firmware_model.yaml)
  - Chu ky 20 ms, chi xu ly lenh khi da ENABLE, watchdog 200 ms, thoat che do ROS sau 1 s im lang
  - joint_stream(): bo lenh ngoai gioi han (bit LIMIT), toc do = quang duong / chu ky x 1.3,
    kep trong [0.5, dps_max], LAM TRON LEN theo don vi ZDT (0.1 RPM, 1 RPM/s)
  - Doc vi tri: lam tron 0.1 do tai khop nhu motor_to_joint()

Bat chuoc driver ZDT (GIA DINH, chua do tren driver that):
  - Lenh FD tuyet doi: chay hinh thang tu VAN TOC HIEN TAI toi dich moi, gia toc = acc, toc do <= vmax.
  - Vi tri dong co lam tron 0.1 do.
Servo MG996R: chay toi do rong xung lenh voi toc do servo_speed_us_s (mac dinh ~0.17 s / 60 do).
"""

import math
import os
import struct

# ----------------------------------------------------------------------
# GIAO THUC (giong ros_link.c va stm32_bridge_node.py)
# ----------------------------------------------------------------------
HEADER = b'\xAA\x55'
T_CMD_JOINT, T_CMD_ENABLE, T_CMD_ESTOP, T_CMD_SET_ZERO, T_CMD_PING, T_CMD_HOME = 0x01, 0x02, 0x03, 0x04, 0x05, 0x06
T_STATE = 0x81
FMT_CMD_JOINT = '<H4iH'
FMT_STATE = '<H4iHHB'

ST_ENABLED, ST_WATCHDOG, ST_CAN_ERROR = 1 << 0, 1 << 1, 1 << 2
ST_NOT_HOMED, ST_LIMIT, ST_ESTOP, ST_HOMING = 1 << 7, 1 << 8, 1 << 9, 1 << 10
HOME_TIME_MS = 5000          # gia lap: home mat 5 s, xong thi 4 khop ve 0 (giua cam bien)


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
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes):
        self.buf += data
        out = []
        while True:
            s = self.buf.find(HEADER)
            if s < 0:
                del self.buf[:-1]
                return out
            del self.buf[:s]
            if len(self.buf) < 4:
                return out
            n = self.buf[3]
            if len(self.buf) < 6 + n:
                return out
            body = bytes(self.buf[2:4 + n])
            if crc16_ccitt(body) == struct.unpack('<H', self.buf[4 + n:6 + n])[0]:
                out.append((body[0], body[2:]))
                del self.buf[:6 + n]
            else:
                del self.buf[:2]


def div_round(a: int, b: int) -> int:
    return (a + b // 2) // b if a >= 0 else -((-a + b // 2) // b)


# ----------------------------------------------------------------------
# MO HINH DRIVER ZDT: lenh FD vi tri tuyet doi, hinh thang tu van toc hien tai
# ----------------------------------------------------------------------
class ZdtModel:
    def __init__(self):
        self.pos = 0.0      # do truc dong co
        self.vel = 0.0      # do/s truc dong co
        self.target = 0.0
        self.vmax = 0.0     # do/s
        self.acc = 1.0      # do/s^2

    def command(self, target_deg, rpm_x10, acc_rpm_s):
        self.target = float(target_deg)
        self.vmax = rpm_x10 / 10.0 * 6.0
        self.acc = max(1.0, acc_rpm_s * 6.0)

    def stop(self):
        self.target = self.pos
        self.vel = 0.0

    def step(self, dt):
        d = self.target - self.pos
        if abs(d) < 1e-6 and abs(self.vel) < 1e-6:
            return
        direction = 1.0 if d > 0 else -1.0
        stop_dist = self.vel * self.vel / (2.0 * self.acc)
        moving_toward = self.vel * direction > 0
        if moving_toward and abs(d) <= stop_dist + 1e-9:
            want = 0.0                                   # phanh de dung dung dich
        else:
            want = direction * self.vmax
        dv = self.acc * dt
        self.vel += max(-dv, min(dv, want - self.vel))
        new = self.pos + self.vel * dt
        # Vuot qua dich khi gan dung: chot tai dich
        if (self.target - self.pos) * (self.target - new) < 0 and abs(self.vel) <= dv * 2:
            new, self.vel = self.target, 0.0
        self.pos = new


# ----------------------------------------------------------------------
# MO HINH FIRMWARE (ros_link.c + joint.c)
# ----------------------------------------------------------------------
class FirmwareModel:
    def __init__(self, cfg):
        self.ratio = [int(v) for v in cfg['ratio']]
        self.dir = [int(v) for v in cfg['dir']]
        self.min_x10 = [int(round(v * 10)) for v in cfg['min_deg']]
        self.max_x10 = [int(round(v * 10)) for v in cfg['max_deg']]
        self.dps = [float(v) for v in cfg['dps']]
        self.accj = [float(v) for v in cfg['acc']]
        self.acc_gain = float(cfg.get('stream_acc_gain', 4.0))
        self.cycle_ms = int(cfg.get('cycle_ms', 20))
        self.watchdog_ms = int(cfg.get('watchdog_ms', 200))
        self.mode_timeout_ms = int(cfg.get('ros_mode_timeout_ms', 1000))
        self.homed = bool(cfg.get('homed', True))
        self.servo_speed = float(cfg.get('servo_speed_us_s', 3900.0))
        self.offset = [0, 0, 0, 0]          # do dong co x10 ung voi goc khop 0
        self.motors = [ZdtModel() for _ in range(4)]
        self.target_x10 = [0, 0, 0, 0]
        self.cmd_x10 = [0, 0, 0, 0]
        self.cmd_us = 0
        self.servo_us = float(cfg.get('servo_start_us', 1300))
        self.servo_target = self.servo_us
        self.have_new = False
        self.active = False
        self.enabled = False
        self.status = 0 if self.homed else ST_NOT_HOMED
        self.home_done_ms = None            # dang home: thoi diem xong
        self.seq = 0
        self.t_ms = 0
        self.last_rx = 0
        self.last_cmd = 0
        self.last_cycle = 0
        self.parser = FrameParser()
        self.limit_events = []              # (thoi gian, khop, goc lenh) de bao cao

    # --- quy doi giong joint.c ---
    def joint_to_motor_x10(self, i, x10):
        return self.offset[i] + self.dir[i] * self.ratio[i] * x10

    def motor_to_joint_x10(self, i, m_x10):
        return div_round((m_x10 - self.offset[i]) * self.dir[i], self.ratio[i])

    def read_pos_x10(self, i):
        m_x10 = int(round(self.motors[i].pos * 10.0))
        return self.motor_to_joint_x10(i, m_x10)

    # --- nhan byte ---
    def feed(self, data: bytes):
        for t, p in self.parser.feed(data):
            self._handle(t, p)

    def _handle(self, t, p):
        self.last_rx = self.t_ms
        if not self.active:
            self.active, self.enabled, self.last_cycle = True, False, self.t_ms
        if t == T_CMD_JOINT and len(p) == 20:
            v = struct.unpack(FMT_CMD_JOINT, p)
            self.seq, self.cmd_x10, self.cmd_us = v[0], list(v[1:5]), v[5]
            self.have_new, self.last_cmd = True, self.t_ms
        elif t == T_CMD_HOME:
            if not self.enabled and self.home_done_ms is None:
                self.home_done_ms = self.t_ms + HOME_TIME_MS
                self.status |= ST_HOMING
                self.status &= ~(ST_ESTOP | ST_WATCHDOG | ST_LIMIT)
        elif t == T_CMD_ENABLE and len(p) == 1:
            if p[0] and (self.status & (ST_NOT_HOMED | ST_HOMING)):
                pass                        # giong firmware: chua home thi khong bat
            elif p[0]:
                for i in range(4):
                    self.target_x10[i] = self.read_pos_x10(i)
                    self.cmd_x10[i] = self.target_x10[i]
                self.status &= ~(ST_WATCHDOG | ST_ESTOP | ST_LIMIT)
                self.have_new, self.last_cmd, self.enabled = False, self.t_ms, True
            else:
                self._stop()
        elif t == T_CMD_ESTOP:
            self._stop()
            self.status |= ST_ESTOP
            if self.home_done_ms is not None:   # huy home
                self.home_done_ms = None
                self.status &= ~ST_HOMING
                self.status |= ST_NOT_HOMED

    def _stop(self):
        for m in self.motors:
            m.stop()
        self.enabled = False

    # --- joint_stream() ---
    def _stream(self, i, x10, dt_ms):
        if x10 < self.min_x10[i] or x10 > self.max_x10[i]:
            self.limit_events.append((self.t_ms / 1000.0, i + 1, x10 / 10.0))
            return False
        diff = x10 - self.target_x10[i]
        if diff == 0:
            return True
        dt_ms = max(5, dt_ms)
        v = abs(diff) / 10.0 / (dt_ms / 1000.0) * 1.3
        v = max(0.5, min(v, self.dps[i]))
        a = self.accj[i] * self.acc_gain
        rpm_x10 = max(1, min(30000, math.ceil(v * self.ratio[i] / 6.0 * 10.0 - 1e-9)))
        acc_rpm = max(1, min(65535, math.ceil(a * self.ratio[i] / 6.0 - 1e-9)))
        self.motors[i].command(self.joint_to_motor_x10(i, x10) / 10.0, rpm_x10, acc_rpm)
        self.target_x10[i] = x10
        return True

    # --- tien thoi gian 1 ms, tra ve khung STATE neu den chu ky ---
    def tick_1ms(self):
        self.t_ms += 1
        for m in self.motors:
            m.step(0.001)
        step = self.servo_speed * 0.001
        self.servo_us += max(-step, min(step, self.servo_target - self.servo_us))

        if not self.active:
            return None
        if self.t_ms - self.last_rx > self.mode_timeout_ms:
            if self.enabled:
                self._stop()
            self.active = False
            return None
        if self.enabled and self.t_ms - self.last_cmd > self.watchdog_ms:
            self._stop()
            self.status |= ST_WATCHDOG
        if self.home_done_ms is not None and self.t_ms >= self.home_done_ms:
            for i in range(4):                  # home xong: khop ve goc 0 (giua cam bien)
                self.motors[i].pos = self.joint_to_motor_x10(i, 0) / 10.0
                self.motors[i].stop()
                self.target_x10[i] = 0
            self.servo_target = 500.0           # home ca tay thi gripper mo
            self.home_done_ms = None
            self.status &= ~(ST_HOMING | ST_NOT_HOMED)
        dt = self.t_ms - self.last_cycle
        if dt < self.cycle_ms:
            return None
        self.last_cycle = self.t_ms

        n_fd = 0
        if self.enabled and self.have_new:
            self.have_new = False
            for i in range(4):
                if self.cmd_x10[i] != self.target_x10[i]:
                    n_fd += 1
                if not self._stream(i, self.cmd_x10[i], dt):
                    self.status |= ST_LIMIT
            if self.cmd_us:
                self.servo_target = float(self.cmd_us)

        # Uoc luong thoi gian vong lap that: moi lenh FD ~1.5 ms, doc vi tri ~1 ms, doc co ~1 ms,
        # gui STATE 29 byte o 115200 baud ~2.5 ms
        loop_ms = int(round(n_fd * 1.5 + 4 * 1.0 + 1.0 + 2.5))
        pos = [self.read_pos_x10(i) for i in range(4)]
        st = self.status | (ST_ENABLED if self.enabled else 0)
        payload = struct.pack(FMT_STATE, self.seq, *pos, int(round(self.servo_us)), st, loop_ms)
        return build_frame(T_STATE, payload)


# ----------------------------------------------------------------------
# NODE ROS 2
# ----------------------------------------------------------------------
def main():
    import pty
    import tty

    import rclpy
    from rclpy.node import Node

    class Stm32Emulator(Node):
        def __init__(self):
            super().__init__('stm32_emulator')
            self.declare_parameter('link_path', '/tmp/ttySTM32')
            for name, default in [('ratio', [12, 27, 14, 1]), ('dir', [1, 1, 1, 1]),
                                  ('min_deg', [-90.0, -90.0, -90.0, -90.0]),
                                  ('max_deg', [90.0, 90.0, 90.0, 90.0]),
                                  ('dps', [80.0, 8.0, 25.0, 80.0]),
                                  ('acc', [120.0, 10.0, 35.0, 200.0])]:
                self.declare_parameter(name, default)
            for name, default in [('stream_acc_gain', 4.0), ('cycle_ms', 20), ('watchdog_ms', 200),
                                  ('ros_mode_timeout_ms', 1000), ('homed', True),
                                  ('servo_speed_us_s', 3900.0), ('servo_start_us', 1300)]:
                self.declare_parameter(name, default)
            cfg = {n: self.get_parameter(n).value for n in
                   ['ratio', 'dir', 'min_deg', 'max_deg', 'dps', 'acc', 'stream_acc_gain', 'cycle_ms',
                    'watchdog_ms', 'ros_mode_timeout_ms', 'homed', 'servo_speed_us_s', 'servo_start_us']}
            self.fw = FirmwareModel(cfg)
            self.n_limit_logged = 0

            # Cong noi tiep ao: node cau noi mo dau "slave" qua duong dan link_path
            self.master, slave = pty.openpty()
            tty.setraw(slave)
            os.set_blocking(self.master, False)
            self.slave_name = os.ttyname(slave)
            self.link = self.get_parameter('link_path').value
            try:
                if os.path.islink(self.link) or os.path.exists(self.link):
                    os.remove(self.link)
                os.symlink(self.slave_name, self.link)
            except OSError as e:
                self.get_logger().error(f'Khong tao duoc {self.link}: {e}')
            self.get_logger().warn(
                f'GIA LAP STM32: cong ao {self.link} -> {self.slave_name}. '
                f'Gioi han khop (do): min {cfg["min_deg"]}, max {cfg["max_deg"]}, toc do {cfg["dps"]}')
            self.wall_prev = self.get_clock().now().nanoseconds
            self.create_timer(0.01, self._tick)    # moi 10 ms chay 10 buoc 1 ms

        def _tick(self):
            try:
                data = os.read(self.master, 4096)
                if data:
                    self.fw.feed(data)
            except BlockingIOError:
                pass
            except OSError:
                pass
            now = self.get_clock().now().nanoseconds
            steps = min(50, max(1, int((now - self.wall_prev) / 1e6)))
            self.wall_prev += steps * 1_000_000
            for _ in range(steps):
                frame = self.fw.tick_1ms()
                if frame:
                    try:
                        os.write(self.master, frame)
                    except OSError:
                        pass
            while self.n_limit_logged < len(self.fw.limit_events):
                t, j, deg = self.fw.limit_events[self.n_limit_logged]
                self.n_limit_logged += 1
                if self.n_limit_logged <= 5 or self.n_limit_logged % 50 == 0:
                    self.get_logger().error(
                        f'[firmware] khop {j} lenh {deg:.1f} do NGOAI gioi han -> bo lenh (bit LIMIT). '
                        f'Tong {self.n_limit_logged} lan')

        def destroy_node(self):
            try:
                if os.path.islink(self.link):
                    os.remove(self.link)
            except OSError:
                pass
            super().destroy_node()

    rclpy.init()
    node = Stm32Emulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
