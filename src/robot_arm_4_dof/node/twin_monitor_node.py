#!/usr/bin/env python3
"""Theo doi song song LENH (Pi gui) va THUC (STM32 tra ve) cho robot that / STM32 gia lap.

Vi tri: robot_arm_4_dof/node/twin_monitor_node.py
Chay tu real_robot.launch.py (twin:=true), khong can chay tay.

Viec node lam:
  1. Phat lai /robot_joint_commands thanh /twin/joint_commands co dau thoi gian moi.
     robot_state_publisher thu 2 (frame_prefix cmd/) dung topic nay ve "bong ma" trong RViz:
     bong ma = noi Pi MUON tay toi, robot dac = noi tay THUC SU dang o.
  2. Ve ban A, ban B, vat 40 mm (so lieu tu worlds/pick_place.sdf) len /twin/scene.
     Vat dinh vao GRIPPER_TCP khi kep dong va TCP o gan vat, roi xuong mat ban khi kep mo.
  3. Tinh sai lech lenh - thuc tung khop, phat /twin/error (do) de ve bang rqt_plot / PlotJuggler.
     Canh bao khi sai lech vuot 50 % / 100 % dung sai bam quy dao cua arm_controller.
  4. Canh bao khi TCP (hoac day vat dang kep) thap hon mat ban, tuc la sap dam ban.
  5. Dong chu tren dau robot: trang thai STM32 va sai lech tung khop.
"""

import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA, Float64MultiArray, UInt16
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

STATUS_BITS = ['ENABLED', 'WATCHDOG', 'CAN_ERROR', 'STALL_J1', 'STALL_J2', 'STALL_J3', 'STALL_J4',
               'NOT_HOMED', 'LIMIT', 'ESTOP']
ERROR_MASK = 0b11_0111_1110


def quat_to_rot(x, y, z, w):
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def rot_to_quat(R):
    tr = R[0][0] + R[1][1] + R[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return ((R[2][1] - R[1][2]) / s, (R[0][2] - R[2][0]) / s, (R[1][0] - R[0][1]) / s, 0.25 * s)
    if R[0][0] > R[1][1] and R[0][0] > R[2][2]:
        s = math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2
        return (0.25 * s, (R[0][1] + R[1][0]) / s, (R[0][2] + R[2][0]) / s, (R[2][1] - R[1][2]) / s)
    if R[1][1] > R[2][2]:
        s = math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2
        return ((R[0][1] + R[1][0]) / s, 0.25 * s, (R[1][2] + R[2][1]) / s, (R[0][2] - R[2][0]) / s)
    s = math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2
    return ((R[0][2] + R[2][0]) / s, (R[1][2] + R[2][1]) / s, 0.25 * s, (R[1][0] - R[0][1]) / s)


def mat_mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mat_vec(A, v):
    return [sum(A[i][k] * v[k] for k in range(3)) for i in range(3)]


def transpose(A):
    return [[A[j][i] for j in range(3)] for i in range(3)]


class TwinMonitor(Node):

    def __init__(self):
        super().__init__('twin_monitor')
        p = self.declare_parameter
        p('arm_joints', ['JOINT_1', 'JOINT_2', 'JOINT_3', 'JOINT_4'])
        p('gripper_joint', 'GRIPPER_JOINT')
        # Phai giong constraints.<khop>.trajectory cua arm_controller trong final.yaml
        p('trajectory_tolerance_rad', [0.10, 0.03, 0.08, 0.10])  # = constraints trong final.yaml
        p('report_period_s', 1.0)
        # Ban: ten va [tam x, y, z, kich thuoc x, y, z] (m), lay tu worlds/pick_place.sdf
        p('box_names', ['table_A', 'table_B'])
        p('box_data', [0.32, 0.0, 0.0975, 0.12, 0.30, 0.195,
                       0.0, -0.32, 0.0975, 0.30, 0.12, 0.195])
        p('cube_xyz', [0.32, 0.0, 0.215])
        p('cube_size', 0.04)
        p('tcp_link', 'GRIPPER_TCP')
        p('world_frame', 'world')
        p('gripper_closed_below_rad', 0.9)   # mo 1.141, kep 0.62
        p('attach_radius_m', 0.04)
        p('penetration_warn_m', 0.005)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.arm = list(g('arm_joints'))
        self.grip = g('gripper_joint')
        self.tol = [float(v) for v in g('trajectory_tolerance_rad')]
        names, data = list(g('box_names')), [float(v) for v in g('box_data')]
        self.boxes = [(n, data[6 * i:6 * i + 3], data[6 * i + 3:6 * i + 6]) for i, n in enumerate(names)]
        self.cube_size = float(g('cube_size'))
        self.cube_pos = [float(v) for v in g('cube_xyz')]
        self.cube_rot = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        self.tcp_link = g('tcp_link')
        self.world = g('world_frame')
        self.closed_below = float(g('gripper_closed_below_rad'))
        self.attach_r = float(g('attach_radius_m'))
        self.pen_warn = float(g('penetration_warn_m'))
        self.report_period = float(g('report_period_s'))

        self.cmd = {}
        self.act = {}
        self.status = None
        self.attached = None          # (R_rel, p_rel) cua vat trong he TCP
        self.window_max = [0.0] * 4   # sai lech lon nhat trong chu ky bao cao
        self.run_max = [0.0] * 4      # tu luc khoi dong
        self.last_report = self.get_clock().now()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub_cmd = self.create_publisher(JointState, '/twin/joint_commands', 10)
        self.pub_err = self.create_publisher(Float64MultiArray, '/twin/error', 10)
        self.pub_scene = self.create_publisher(
            MarkerArray, '/twin/scene', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(JointState, '/robot_joint_commands', self._on_cmd, 10)
        self.create_subscription(JointState, '/robot_joint_states', self._on_state, 10)
        self.create_subscription(UInt16, '/stm32_bridge/status', self._on_status, 10)
        self.create_timer(0.05, self._tick)

    # ---------------- nhan du lieu ----------------

    def _on_cmd(self, msg):
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        for n, v in zip(msg.name, msg.position):
            if math.isfinite(v):
                self.cmd[n] = v
                out.name.append(n)
                out.position.append(v)
        if out.name:
            self.pub_cmd.publish(out)

    def _on_state(self, msg):
        for n, v in zip(msg.name, msg.position):
            self.act[n] = v

    def _on_status(self, msg):
        if self.status is not None and msg.data != self.status:
            names = [n for b, n in enumerate(STATUS_BITS) if msg.data & (1 << b)]
            _msg = f'STM32 doi trang thai: {names or ["OFF"]}'
            if msg.data & ERROR_MASK:
                self.get_logger().error(_msg)
            else:
                self.get_logger().info(_msg)
        self.status = msg.data

    # ---------------- vong lap ----------------

    def _tcp(self):
        try:
            t = self.tf_buffer.lookup_transform(self.world, self.tcp_link, Time(),
                                                timeout=Duration(seconds=0.0))
        except Exception:
            return None
        tr, q = t.transform.translation, t.transform.rotation
        return quat_to_rot(q.x, q.y, q.z, q.w), [tr.x, tr.y, tr.z]

    def _tick(self):
        err = None
        if all(j in self.cmd and j in self.act for j in self.arm):
            err = [abs(self.cmd[j] - self.act[j]) for j in self.arm]
            self.window_max = [max(a, b) for a, b in zip(self.window_max, err)]
            self.run_max = [max(a, b) for a, b in zip(self.run_max, err)]
            self.pub_err.publish(Float64MultiArray(data=[math.degrees(e) for e in err]))

        tcp = self._tcp()
        if tcp is not None:
            self._update_cube(*tcp)
            self._check_clearance(tcp[1])

        now = self.get_clock().now()
        if (now - self.last_report).nanoseconds * 1e-9 >= self.report_period:
            self.last_report = now
            self._report()
        self._publish_scene(err)

    def _update_cube(self, R, p):
        grip = self.act.get(self.grip)
        if grip is None:
            return
        closed = grip < self.closed_below
        if self.attached is None and closed:
            d = math.dist(p, self.cube_pos)
            if d < self.attach_r:
                Rt = transpose(R)
                self.attached = (mat_mul(Rt, self.cube_rot),
                                 mat_vec(Rt, [c - q for c, q in zip(self.cube_pos, p)]))
                self.get_logger().info(f'Da kep vat (TCP cach tam vat {d * 1000:.0f} mm)')
        elif self.attached is not None and not closed:
            self.attached = None
            self._drop_cube()
            self.get_logger().info(
                f'Da tha vat tai x {self.cube_pos[0]:.3f} y {self.cube_pos[1]:.3f} z {self.cube_pos[2]:.3f} m')
        if self.attached is not None:
            R_rel, p_rel = self.attached
            self.cube_rot = mat_mul(R, R_rel)
            self.cube_pos = [a + b for a, b in zip(p, mat_vec(R, p_rel))]

    def _drop_cube(self):
        """Vat roi thang dung xuong mat ban ben duoi (hoac san), giu goc xoay quanh truc z."""
        h = self.cube_size / 2.0
        floor = 0.0
        for _, c, s in self.boxes:
            if abs(self.cube_pos[0] - c[0]) <= s[0] / 2 and abs(self.cube_pos[1] - c[1]) <= s[1] / 2:
                floor = max(floor, c[2] + s[2] / 2)
        self.cube_pos[2] = floor + h
        yaw = math.atan2(self.cube_rot[1][0], self.cube_rot[0][0])
        c, s = math.cos(yaw), math.sin(yaw)
        self.cube_rot = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]

    def _check_clearance(self, p):
        pts = [('TCP', p)]
        if self.attached is not None:
            pts.append(('day vat', [self.cube_pos[0], self.cube_pos[1], self.cube_pos[2] - self.cube_size / 2]))
        for label, q in pts:
            for name, c, s in self.boxes:
                if abs(q[0] - c[0]) <= s[0] / 2 and abs(q[1] - c[1]) <= s[1] / 2:
                    pen = c[2] + s[2] / 2 - q[2]
                    if pen > self.pen_warn:
                        self.get_logger().error(
                            f'{label} thap hon mat {name} {pen * 1000:.0f} mm: SAP DAM BAN',
                            throttle_duration_sec=1.0)

    def _report(self):
        if not any(self.window_max):
            return
        ratio = [e / t for e, t in zip(self.window_max, self.tol)]
        worst = max(range(4), key=lambda i: ratio[i])
        txt = '  '.join(f'J{i + 1} {math.degrees(e):.2f}' for i, e in enumerate(self.window_max))
        if ratio[worst] > 1.0:
            self.get_logger().error(f'Sai lech lenh-thuc (do) {txt}: khop {worst + 1} VUOT dung sai '
                                    f'{math.degrees(self.tol[worst]):.2f} do, controller se huy quy dao')
        elif ratio[worst] > 0.5:
            self.get_logger().warn(f'Sai lech lenh-thuc (do) {txt}: khop {worst + 1} da qua 50 % dung sai')
        self.window_max = [0.0] * 4

    # ---------------- ve ----------------

    def _box(self, mid, ns, pos, size, rgba, quat=(0.0, 0.0, 0.0, 1.0)):
        m = Marker()
        m.header.frame_id = self.world
        m.ns, m.id, m.type, m.action = ns, mid, Marker.CUBE, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = pos
        (m.pose.orientation.x, m.pose.orientation.y,
         m.pose.orientation.z, m.pose.orientation.w) = quat
        m.scale.x, m.scale.y, m.scale.z = size
        m.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
        return m

    def _publish_scene(self, err):
        arr = MarkerArray()
        for i, (name, c, s) in enumerate(self.boxes):
            arr.markers.append(self._box(i, 'ban', c, s, (0.55, 0.40, 0.25, 0.9)))
        cube_color = (0.1, 0.8, 0.2, 1.0) if self.attached else (0.85, 0.15, 0.15, 1.0)
        arr.markers.append(self._box(0, 'vat', self.cube_pos, [self.cube_size] * 3, cube_color,
                                     rot_to_quat(self.cube_rot)))

        t = Marker()
        t.header.frame_id = self.world
        t.ns, t.id, t.type, t.action = 'trang_thai', 0, Marker.TEXT_VIEW_FACING, Marker.ADD
        t.pose.position = Point(x=0.0, y=0.0, z=0.75)
        t.pose.orientation.w = 1.0
        t.scale.z = 0.025
        if self.status is None:
            head, bad = 'STM32: chua co trang thai', True
        else:
            names = [n for b, n in enumerate(STATUS_BITS) if self.status & (1 << b)]
            head, bad = 'STM32: ' + (' '.join(names) or 'OFF'), bool(self.status & ERROR_MASK)
        lines = [head]
        if err is not None:
            lines.append('lech (do) ' + '  '.join(f'J{i + 1} {math.degrees(e):.2f}' for i, e in enumerate(err)))
            lines.append('max (do)  ' + '  '.join(f'J{i + 1} {math.degrees(e):.2f}' for i, e in enumerate(self.run_max)))
        t.text = '\n'.join(lines)
        t.color = ColorRGBA(r=0.9, g=0.1, b=0.1, a=1.0) if bad else ColorRGBA(r=0.1, g=0.6, b=0.1, a=1.0)
        arr.markers.append(t)
        self.pub_scene.publish(arr)


def main():
    rclpy.init()
    node = TwinMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()