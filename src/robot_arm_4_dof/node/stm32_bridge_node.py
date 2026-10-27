#!/usr/bin/env python3
"""Cau noi ROS 2 <-> STM32 (firmware robot_arm_firmware_stm32, module ros_link.c).

Vi tri: robot_arm_4_dof/node/stm32_bridge_node.py

Luong du lieu:
  JointStateTopicSystem --/robot_joint_commands (rad)--> node nay --USART--> STM32 --CAN--> ZDT
  JointStateTopicSystem <--/robot_joint_states  (rad)--- node nay <--USART-- STM32 <--CAN-- ZDT

Ket noi: Pi <-> STM32 USART1 (PA9 TX, PA10 RX, 115200 8N1), qua bo chuyen USB-UART 3.3 V
(cong /dev/ttyUSB0) hoac UART GPIO cua Pi (/dev/ttyAMA0). USART1 dung chung voi console go lenh:
STM32 tu chuyen sang che do nhi phan khi nhan goi dau tien, 1 s khong co goi thi tro lai console.

DOI DON VI (node nay lam, firmware khong biet URDF):
  Khop:  goc_firmware_do = joint_sign * deg(goc_URDF_rad) + joint_offset_deg
         Firmware dung 0.1 do theo bang joint.c. Goc 0 firmware = GIUA CAM BIEN HANH TRINH
         (home.c, 4 den opto sang). Neu URDF da sua de tu the 0 trung tu the do va cung chieu
         thi joint_sign = 1, joint_offset_deg = 0.
  Kep:   GRIPPER_JOINT (rad) -> khe ho = 2 * gripper_crank_m * sin(q) -> do rong xung servo (us)
         theo bang do thuc te gripper_gap_mm <-> gripper_us (noi suy tuyen tinh).

Hai che do:
  loopback:=true   Gia lap STM32 trong node (khong can phan cung), van di qua buoc doi don vi.
  loopback:=false  Doc ghi cong noi tiep that.

HOME: goi ~/home (STM32 chay home bang cam bien, kem mo gripper, khoang 30 s), hoac go "home"
  tren console STM32 truoc khi chay node. Chua home (NOT_HOMED) thi ~/enable bi tu choi,
  firmware cung tu choi.

AN TOAN:
  - Mac dinh TAT. Goi dich vu ~/enable de STM32 nhan lenh chuyen dong.
  - Chan lenh nhay qua max_jump_rad so voi vi tri thuc.
  - Khi TAT van gui PING moi chu ky de STM32 o lai che do nhi phan va gui trang thai.
  - STM32 bao loi (watchdog, CAN, ket truc, ngoai gioi han, dung khan cap) thi tu TAT.
"""

import math
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt16
from std_srvs.srv import SetBool, Trigger

# ----------------------------------------------------------------------
# GIAO THUC: PHAI TRUNG VOI ros_link.h / ros_link.c
# ----------------------------------------------------------------------
HEADER = b'\xAA\x55'

T_CMD_JOINT = 0x01      # seq u16 | 4 x i32 (0.1 do firmware) | servo_us u16          = 20 byte
T_CMD_ENABLE = 0x02     # on u8
T_CMD_ESTOP = 0x03      # rong
T_CMD_SET_ZERO = 0x04   # mask u8 (bit0 = khop 1 ... bit3 = khop 4)
T_CMD_PING = 0x05       # rong
T_CMD_HOME = 0x06       # rong. Home bang cam bien, chi khi dang TAT
T_STATE = 0x81          # seq u16 | 4 x i32 | servo_us u16 | status u16 | loop_ms u8  = 23 byte

FMT_CMD_JOINT = '<H4iH'
FMT_STATE = '<H4iHHB'

STATUS_BITS = {
    0: 'ENABLED', 1: 'WATCHDOG', 2: 'CAN_ERROR',
    3: 'STALL_J1', 4: 'STALL_J2', 5: 'STALL_J3', 6: 'STALL_J4',
    7: 'NOT_HOMED', 8: 'LIMIT', 9: 'ESTOP', 10: 'HOMING',
}
ST_ENABLED = 1 << 0
ST_NOT_HOMED = 1 << 7
ST_HOMING = 1 << 10
ERROR_MASK = 0b011_0111_1110  # moi bit tru ENABLED, NOT_HOMED, HOMING


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
    """Tach khung tu dong byte. Bo qua chu console in ra truoc khi STM32 vao che do nhi phan."""

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
                del self.buf[:2]


def interp(x, xp, fp):
    """Noi suy tuyen tinh, xp tang dan, ngoai khoang thi kep o 2 dau."""
    if x <= xp[0]:
        return fp[0]
    if x >= xp[-1]:
        return fp[-1]
    for i in range(1, len(xp)):
        if x <= xp[i]:
            t = (x - xp[i - 1]) / (xp[i] - xp[i - 1])
            return fp[i - 1] + t * (fp[i] - fp[i - 1])
    return fp[-1]


# ----------------------------------------------------------------------
# NODE
# ----------------------------------------------------------------------
class Stm32Bridge(Node):

    def __init__(self):
        super().__init__('stm32_bridge')
        p = self.declare_parameter
        p('port', '/dev/ttyUSB0')
        p('baud', 115200)
        p('rate_hz', 50.0)
        p('loopback', False)
        p('auto_enable', False)            # chi dung voi loopback / STM32 gia lap, KHONG dung voi robot that
        p('arm_joints', ['JOINT_1', 'JOINT_2', 'JOINT_3', 'JOINT_4'])
        p('gripper_joint', 'GRIPPER_JOINT')
        # Doi URDF -> firmware cho tung khop. PHAI HIEU CHUAN tren robot that (xem README)
        p('joint_sign', [1.0, 1.0, 1.0, 1.0])
        p('joint_offset_deg', [0.0, 0.0, 0.0, 0.0])
        # Kep: bang do thuc te (s 500 = 53 mm, s 1000 = 23 mm, s 1270 = 2 ngon vua cham = GRIP_CLOSE_US)
        p('gripper_crank_m', 0.033)
        p('gripper_gap_mm', [53.0, 23.0, 0.0])
        p('gripper_us', [500.0, 1000.0, 1270.0])
        p('max_jump_rad', 0.2)
        p('state_timeout_s', 0.5)
        p('loopback_speed_deg_s', 60.0)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.port, self.baud, self.rate = g('port'), g('baud'), g('rate_hz')
        self.loopback = g('loopback')
        self.arm_joints = list(g('arm_joints'))
        self.grip_joint = g('gripper_joint')
        self.joints = self.arm_joints + [self.grip_joint]
        self.sign = [float(v) for v in g('joint_sign')]
        self.offset = [float(v) for v in g('joint_offset_deg')]
        self.crank = g('gripper_crank_m')
        pairs = sorted(zip(g('gripper_gap_mm'), g('gripper_us')))   # gap tang dan
        self.gap_pts = [a for a, _ in pairs]
        self.us_pts = [b for _, b in pairs]
        self.max_jump = g('max_jump_rad')
        self.state_timeout = g('state_timeout_s')
        self.lb_speed = g('loopback_speed_deg_s')

        self.cmd = None          # lenh moi nhat (rad, thu tu self.joints)
        self.grip_topic = None   # lenh gripper moi nhat tu ROS (rad)
        self.grip_hold = None    # lenh gripper luc enable: ROS chua doi lenh thi gui 0 us (giu nguyen servo)
        self.state = None        # trang thai moi nhat (rad)
        self.status = 0
        self.enabled = False
        self.seq = 0
        self.last_state_time = None
        self.parser = FrameParser()
        self.serial = None
        self.lb_fw = [0, 0, 0, 0]          # loopback: goc firmware (0.1 do)
        self.lb_us = int(self.us_pts[-1])  # loopback: servo (us)
        self.lb_enabled = False
        self.lb_homed = True             # loopback: gia dinh da home
        self.enable_grace_until = None   # vua bat: cho STM32 xac nhan ENABLED
        self.auto_enable_pending = bool(g('auto_enable'))
        self.next_open_try = 0           # mo lai cong noi tiep moi 1 s neu chua mo duoc

        if self.loopback:
            self.get_logger().warn('CHE DO LOOPBACK: gia lap STM32, khong dieu khien phan cung that')
        else:
            self._open_serial()

        self.pub_state = self.create_publisher(JointState, '/robot_joint_states', 10)
        self.pub_status = self.create_publisher(UInt16, '~/status', 10)
        self.create_subscription(JointState, '/robot_joint_commands', self._on_cmd, 10)
        self.create_service(SetBool, '~/enable', self._srv_enable)
        self.create_service(Trigger, '~/estop', self._srv_estop)
        self.create_service(Trigger, '~/home', self._srv_home)
        self.create_timer(1.0 / self.rate, self._tick)


    # ---------------- doi don vi ----------------

    def ros_to_fw(self, q_rad):
        """4 goc khop (rad URDF) -> 4 goc firmware (int 0.1 do)."""
        return [int(round((s * math.degrees(q) + o) * 10.0))
                for q, s, o in zip(q_rad, self.sign, self.offset)]

    def fw_to_ros(self, x10):
        return [math.radians((v / 10.0 - o) * s) for v, s, o in zip(x10, self.sign, self.offset)]

    def grip_rad_to_us(self, q):
        gap = 2000.0 * self.crank * math.sin(max(0.0, min(q, math.pi / 2)))
        return int(round(interp(gap, self.gap_pts, self.us_pts)))

    def grip_us_to_rad(self, us):
        if us == 0:
            return 0.0
        us_sorted = sorted(zip(self.us_pts, self.gap_pts))
        gap = interp(us, [a for a, _ in us_sorted], [b for _, b in us_sorted])
        return math.asin(max(0.0, min(1.0, gap / (2000.0 * self.crank))))

    # ---------------- cong noi tiep ----------------

    def _open_serial(self):
        import serial  # python3-serial
        try:
            self.serial = serial.Serial(self.port, baudrate=self.baud, timeout=0)
            self.serial.reset_input_buffer()
            self.get_logger().info(f'Da mo {self.port} @ {self.baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'Khong mo duoc {self.port}: {e}. Thu lai moi 1 s',
                                    throttle_duration_sec=5.0)
            self.serial = None

    def _send(self, msg_type, payload=b''):
        if self.loopback:
            self._loopback_rx(msg_type, payload)
            return
        if self.serial is None:
            return
        try:
            self.serial.write(build_frame(msg_type, payload))
        except Exception as e:
            self.get_logger().error(f'Loi ghi cong noi tiep: {e}')
            self.serial = None

    def _read_frames(self):
        if self.loopback:
            return [(T_STATE, self._loopback_state())]
        if self.serial is None:
            return []
        try:
            data = self.serial.read(self.serial.in_waiting or 0)
        except Exception as e:
            self.get_logger().error(f'Loi doc cong noi tiep: {e}')
            self.serial = None
            return []
        return self.parser.feed(data) if data else []

    # ---------------- callback ----------------

    def _on_cmd(self, msg: JointState):
        # Bo gia tri NaN / inf (lenh chua khoi tao truoc khi controller chay)
        pos = {n: v for n, v in zip(msg.name, msg.position) if math.isfinite(v)}
        if self.cmd is None:
            self.cmd = list(self.state) if self.state is not None else [0.0] * len(self.joints)
        for i, j in enumerate(self.joints):
            if j in pos:
                self.cmd[i] = pos[j]
        if self.joints[4] in pos:
            self.grip_topic = pos[self.joints[4]]

    def _try_auto_enable(self):
        """Loopback / STM32 gia lap: tu bat dong co khi da co trang thai dau tien."""
        if not self.auto_enable_pending or self.state is None:
            return
        self.auto_enable_pending = False
        req, res = SetBool.Request(data=True), SetBool.Response()
        self._srv_enable(req, res)

    def _srv_enable(self, req, res):
        if req.data and self.state is None:
            res.success, res.message = False, 'Chua nhan trang thai tu STM32, khong bat'
            return res
        if req.data and self.status & ST_HOMING:
            res.success, res.message = False, 'STM32 dang home, cho xong roi bat'
            return res
        if req.data and self.status & ST_NOT_HOMED:
            res.success, res.message = False, 'STM32 chua home (NOT_HOMED). Goi ~/home truoc'
            self.get_logger().error(res.message)
            return res
        if req.data:
            self.cmd = list(self.state)        # lenh = vi tri thuc, khong giat
            self.grip_hold = self.grip_topic   # lenh gripper cu (truoc home) khong duoc dong gripper lai
        self._send(T_CMD_ENABLE, b'\x01' if req.data else b'\x00')
        self.enabled = req.data
        self.enable_grace_until = self.get_clock().now().nanoseconds + int(0.3e9)
        res.success, res.message = True, 'BAT' if req.data else 'TAT'
        self.get_logger().info(f'Dong co: {res.message}')
        return res

    def _srv_estop(self, req, res):
        self.enabled = False
        self._send(T_CMD_ESTOP)
        self.get_logger().error('DUNG KHAN CAP (dong co van giu luc)')
        res.success, res.message = True, 'ESTOP'
        return res

    def _srv_home(self, req, res):
        if self.enabled:
            res.success, res.message = False, 'TAT truoc khi home'
            return res
        if self.state is None:
            res.success, res.message = False, 'Chua nhan trang thai tu STM32'
            return res
        self._send(T_CMD_HOME)
        res.success, res.message = True, 'Da gui lenh home (khoang 30 s). Theo doi ~/status: HOMING roi het NOT_HOMED'
        self.get_logger().info(res.message)
        return res

    # ---------------- vong lap chinh ----------------

    def _tick(self):
        now = self.get_clock().now()

        # 0. Chua mo duoc cong (rut cap USB, STM32 gia lap chua tao cong): thu lai moi 1 s
        if not self.loopback and self.serial is None and now.nanoseconds >= self.next_open_try:
            self.next_open_try = now.nanoseconds + int(1e9)
            self._open_serial()
            self.parser = FrameParser()

        # 1. Nhan trang thai
        for msg_type, payload in self._read_frames():
            if msg_type != T_STATE or len(payload) != struct.calcsize(FMT_STATE):
                continue
            v = struct.unpack(FMT_STATE, payload)
            q = self.fw_to_ros(v[1:5]) + [self.grip_us_to_rad(v[5])]
            self.state = q
            self._update_status(v[6])
            if v[7] > 1000.0 / self.rate:
                self.get_logger().warn(
                    f'Vong lap STM32 mat {v[7]} ms, cham hon chu ky {1000.0 / self.rate:.0f} ms',
                    throttle_duration_sec=5.0)
            self.last_state_time = now

        self._try_auto_enable()

        # 2. Mat trang thai qua lau
        if self.last_state_time is not None and self.enabled:
            age = (now - self.last_state_time).nanoseconds * 1e-9
            if age > self.state_timeout:
                self.enabled = False
                self.get_logger().error(f'Mat trang thai STM32 {age:.2f} s, tu TAT')

        # 3. Phat trang thai cho ros2_control
        if self.state is not None:
            js = JointState()
            js.header.stamp = now.to_msg()
            js.name = self.joints
            js.position = list(self.state)
            self.pub_state.publish(js)

        # 4. Gui lenh (hoac PING de giu ket noi)
        if not self.enabled or self.cmd is None or self.state is None:
            self._send(T_CMD_PING)
            return
        jump = max(abs(c - s) for c, s in zip(self.cmd[:4], self.state[:4]))
        if jump > self.max_jump:
            self.get_logger().error(
                f'Lenh lech vi tri thuc {jump:.3f} rad > {self.max_jump} rad, KHONG gui. '
                'Kiem tra controller co dang giu vi tri cu, hoac joint_sign / joint_offset_deg.',
                throttle_duration_sec=1.0)
            self._send(T_CMD_PING)
            return
        self.seq = (self.seq + 1) & 0xFFFF
        fw = self.ros_to_fw(self.cmd[:4])
        if (self.grip_hold is not None and self.grip_topic is not None
                and abs(self.grip_topic - self.grip_hold) < 1e-4):
            us = 0                             # firmware: 0 us = giu nguyen servo
        else:
            self.grip_hold = None
            us = self.grip_rad_to_us(self.cmd[4])
        self._send(T_CMD_JOINT, struct.pack(FMT_CMD_JOINT, self.seq, *fw, us))

    def _update_status(self, status):
        if status != self.status:
            names = [n for b, n in STATUS_BITS.items() if status & (1 << b)]
            _msg = f'Trang thai STM32: {names or ["OK"]}'
            if status & ERROR_MASK:
                self.get_logger().error(_msg)
            else:
                self.get_logger().info(_msg)
        self.status = status
        self.pub_status.publish(UInt16(data=status))
        in_grace = (self.enable_grace_until is not None
                    and self.get_clock().now().nanoseconds < self.enable_grace_until)
        if self.enabled and (status & ERROR_MASK or (not status & ST_ENABLED and not in_grace)):
            self.enabled = False
            self.get_logger().error('STM32 bao loi hoac da tu dung, TAT gui lenh. Goi lai ~/enable sau khi xu ly.')

    # ---------------- gia lap STM32 (loopback) ----------------

    def _loopback_rx(self, msg_type, payload):
        if msg_type == T_CMD_HOME and not self.lb_enabled:
            self.lb_fw, self.lb_us, self.lb_homed = [0, 0, 0, 0], int(self.us_pts[0]), True
        elif msg_type == T_CMD_ENABLE:
            self.lb_enabled = payload == b'\x01' and self.lb_homed
        elif msg_type == T_CMD_ESTOP:
            self.lb_enabled = False
        elif msg_type == T_CMD_JOINT and self.lb_enabled:
            v = struct.unpack(FMT_CMD_JOINT, payload)
            step = int(self.lb_speed * 10.0 / self.rate)
            self.lb_fw = [c + max(-step, min(step, t - c)) for c, t in zip(self.lb_fw, v[1:5])]
            if v[5]:
                self.lb_us = v[5]

    def _loopback_state(self):
        status = (ST_ENABLED if self.lb_enabled else 0) | (0 if self.lb_homed else ST_NOT_HOMED)
        return struct.pack(FMT_STATE, self.seq, *self.lb_fw, self.lb_us, status, 1)


def main():
    rclpy.init()
    node = Stm32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._send(T_CMD_ENABLE, b'\x00')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
