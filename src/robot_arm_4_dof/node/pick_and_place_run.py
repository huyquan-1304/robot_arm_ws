#!/usr/bin/env python3
"""Pick and place cho robot_arm_4_dof trong Gazebo Harmonic (ROS 2 Jazzy).

Gap khoi lap phuong 40 mm tren be A (truoc mat), xoay 90 do sang phai o tu the carry,
dat len be B, roi ve home.

[03/11/2026] Co khop 5 (servo xoay gripper, wrist_controller):
  - obj_yaw_deg: vat o A xoay bao nhieu do so voi huong chuan (0 = nhu cu). place_yaw_deg tuong tu o B.
  - Tu the gap / dat tinh bang IK 5 khop tu URDF luc chay, BU do lech 18 mm giua TCP va truc JOINT_5.
  - Vi du: ros2 run robot_arm_4_dof pick_and_place_run.py --ros-args -p obj_yaw_deg:=30.0

Tham so sequence:
  'front' (mac dinh): gap vat o A (J5 xoay theo vat), nhac thang len PRE_A, J5 ve 0, thu ve CARRY_F (J2 = 0)
                      giu hold_s giay, ra PRE_P, dat lai DUNG CHO CU voi J5 = 0 (vat duoc nan thang), ve home.
                      CARRY_F: J2 dung nhu home de khi xe chay J2 it chiu tai, vat cao hon lidar.
  'ab'              : chuoi cu, gap o A, xoay J1 90 do, dat o B.

Cach chay (sau khi pick_and_place_sim.launch.py da khoi dong xong, RViz da hien):
  source ~/ros2_ws/install/setup.bash
  python3 ~/ros2_ws/src/robot_arm_4_dof/scripts/pick_place_node.py

Cach dieu khien: gui quy dao goc khop truc tiep toi arm_controller va gripper_controller
(action FollowJointTrajectory). Quy dao noi suy thang trong khong gian khop giua cac diem dung.
Da kiem tra bang dong hoc thuan tu URDF:
  - Kep luon huong thang xuong tren moi doan (nghieng 0 do).
  - Doan ha va nhac lech ngang toi da 4 mm, ke ho kep mo moi ben 10 mm.
  - Doan xoay giu nguyen do cao 0.32 m va ban kinh 0.20 m, khong cham be (be cach truc J1 tu 0.26 m).
"""

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node

from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint

# ---------------------------------------------------------------------------
# DONG HOC TU URDF (chi dung numpy). Doc /robot_description, tinh FK va IK cho 5 khop
# J1..J4 + JOINT_5 de dua TCP toi diem can, kep thang xuong, goc xoay kep theo yeu cau.
# Bu san do lech 18 mm giua TCP va truc JOINT_5: khi JOINT_5 xoay, IK tu doi J1..J4.
# ---------------------------------------------------------------------------
import xml.etree.ElementTree as ET

import numpy as np

IK_JOINTS = ['JOINT_1', 'JOINT_2', 'JOINT_3', 'JOINT_4', 'JOINT_5']


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis_rot(ax, q):
    ax = np.asarray(ax, float) / np.linalg.norm(ax)
    k = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    return np.eye(3) + np.sin(q) * k + (1 - np.cos(q)) * k @ k


class UrdfChain:
    """Chuoi khop tu goc URDF toi tip (mac dinh GRIPPER_TCP)."""

    def __init__(self, urdf_xml, tip='GRIPPER_TCP'):
        root = ET.fromstring(urdf_xml)
        by_child = {j.find('child').get('link'): j for j in root.findall('joint')}
        chain, link = [], tip
        while link in by_child:
            chain.append(by_child[link])
            link = by_child[link].find('parent').get('link')
        self.steps = []
        self.limits = {}
        for j in reversed(chain):
            o = j.find('origin')
            xyz = [float(v) for v in o.get('xyz', '0 0 0').split()] if o is not None else [0, 0, 0]
            rpy = [float(v) for v in o.get('rpy', '0 0 0').split()] if o is not None else [0, 0, 0]
            t = np.eye(4)
            t[:3, :3] = _rpy(*rpy)
            t[:3, 3] = xyz
            name = j.get('name')
            ax = None
            if j.get('type') in ('revolute', 'continuous'):
                ax = [float(v) for v in j.find('axis').get('xyz').split()]
                lim = j.find('limit')
                if lim is not None:
                    self.limits[name] = (float(lim.get('lower')), float(lim.get('upper')))
            self.steps.append((name, t, ax))

    def fk(self, q):
        """q: dict ten khop -> rad. Tra ve ma tran 4x4 cua tip trong he goc."""
        a = np.eye(4)
        for name, t, ax in self.steps:
            a = a @ t
            if ax is not None:
                r = np.eye(4)
                r[:3, :3] = _axis_rot(ax, q.get(name, 0.0))
                a = a @ r
        return a


def _wrap_half_pi(a):
    """Dua goc ve [-pi/2, pi/2): gripper 2 ngon doi xung 180 do."""
    return (a + np.pi / 2) % np.pi - np.pi / 2


def grip_yaw(t):
    """Huong mo/dong cua 2 ngon (truc z cua TCP) chieu len mat ban, rad."""
    return float(np.arctan2(t[1, 2], t[0, 2]))


def solve_ik(chain, p_target, yaw_target, seed, iters=200):
    """IK 5 khop: TCP toi p_target (m), kep thang xuong, huong mo ngon = yaw_target (mod 180 do).
    seed: list 5 goc (rad). Tra ve (q list, sai so vi tri mm, sai so nghieng do, sai so yaw do)."""
    lo = np.array([chain.limits.get(n, (-np.pi, np.pi))[0] for n in IK_JOINTS])
    hi = np.array([chain.limits.get(n, (-np.pi, np.pi))[1] for n in IK_JOINTS])
    p_target = np.asarray(p_target, float)

    def res(x):
        t = chain.fk(dict(zip(IK_JOINTS, x)))
        a = 2.0 * np.arctan2(t[1, 2], t[0, 2])
        return np.r_[(t[:3, 3] - p_target) * 1000.0,
                     (t[:3, 0] - [0.0, 0.0, -1.0]) * 200.0,
                     np.array([np.cos(a) - np.cos(2 * yaw_target), np.sin(a) - np.sin(2 * yaw_target)]) * 100.0]

    x = np.clip(np.asarray(seed, float), lo, hi)
    lam = 1e-3
    r = res(x)
    for _ in range(iters):
        jac = np.zeros((len(r), len(x)))
        for i in range(len(x)):
            d = np.zeros(len(x))
            d[i] = 1e-6
            jac[:, i] = (res(x + d) - r) / 1e-6
        h = jac.T @ jac
        step = np.linalg.solve(h + lam * np.diag(np.diag(h) + 1e-9), -jac.T @ r)
        xn = np.clip(x + step, lo, hi)
        rn = res(xn)
        if rn @ rn < r @ r:
            x, r, lam = xn, rn, lam * 0.3
            if np.linalg.norm(step) < 1e-10:
                break
        else:
            lam *= 10.0
            if lam > 1e8:
                break
    t = chain.fk(dict(zip(IK_JOINTS, x)))
    pos_err = float(np.linalg.norm(t[:3, 3] - p_target) * 1000.0)
    tilt_err = float(np.degrees(np.arccos(np.clip(-t[2, 0], -1.0, 1.0))))
    yaw_err = float(np.degrees(abs(_wrap_half_pi(grip_yaw(t) - yaw_target))))
    return [float(v) for v in x], pos_err, tilt_err, yaw_err


def plan_tcp(chain, p_target, yaw_target, seed4):
    """Thu vai diem xuat phat cho JOINT_5, lay nghiem tot nhat.
    Kep chi thang xuong: JOINT_5 duong lam huong ngon quay NGUOC chieu yaw nhin tu tren."""
    q0 = chain.fk(dict(zip(IK_JOINTS, list(seed4) + [0.0])))
    d = _wrap_half_pi(yaw_target - grip_yaw(q0))
    best = None
    for q5 in (-d, -d + np.pi, -d - np.pi, 0.0):
        if abs(q5) > np.pi / 2 + 1e-6:
            continue
        sol = solve_ik(chain, p_target, yaw_target, list(seed4) + [q5])
        score = sol[1] + sol[2] + sol[3]
        if best is None or score < best[1] + best[2] + best[3]:
            best = sol
    return best


ARM_JOINTS = ['JOINT_1', 'JOINT_2', 'JOINT_3', 'JOINT_4']
GRIPPER_JOINT = 'GRIPPER_JOINT'
WRIST_JOINT = 'JOINT_5'
WRIST_SPEED = 1.5      # rad/s, duoi 120 do/s cua firmware (WRIST_DPS)

# Diem gap / dat (m, he world). Do cao giu nhu bang POSES ben duoi
Z_PRE, Z_GRASP, Z_PLACE = 0.260, 0.175, 0.177   # be cao H = 0.175 m

# Goc khop (rad) cho tung diem dung, tinh bang dong hoc nguoc tu URDF, kep huong thang xuong
# [03/11/2026] Tinh lai cho khau 4 moi + JOINT_5 (JOINT_5 = 0): TCP va huong kep giu nhu cu, sai so < 0.01 mm
POSES = {
    'HOME':    [0.0,     0.0,     0.0,     0.0],
    'PRE_A':   [-0.0232, -0.6091, -0.8493, -0.2402],  # TCP (0.320, 0, 0.280)
    'GRASP_A': [-0.0232, -0.7515, -0.5729, 0.1786],   # TCP (0.320, 0, 0.195)
    'CARRY_A': [-0.0371, 0.0301,  -0.2344, -0.2645],  # TCP (0.200, 0, 0.320), day vat tren lidar
    'CARRY_B': [1.5337,  0.0301,  -0.2344, -0.2645],  # nhu CARRY_A, chi xoay J1
    # [03/11] Tu the giu vat khi xe chay (chuoi front): J2 = 0 nhu home, kep thang xuong, J5 = 0.
    # TCP (0.2065, 0, 0.328): cao hon CARRY_A 8 mm. J1 lay theo PRE_A luc chay.
    'CARRY_F': [-0.0232, 0.0,     -0.3100, -0.3100],
    'PRE_B':   [1.5476,  -0.6091, -0.8493, -0.2402],  # TCP (0, -0.320, 0.280)
    'PLACE_B': [1.5476,  -0.7463, -0.5766, 0.1697],   # TCP (0, -0.320, 0.197), cach mat be 2 mm
}

GRIPPER_OPEN = 1.141   # khe ho 60 mm
GRIPPER_GRASP = 0.62   # tiep xuc vat 40 mm o 0.651, ep them khoang 0.85 mm moi ben

# Toc do khop dinh (rad/s). Thap hon gioi han 1.0 rad/s trong joint_limits.yaml
SPEED_NORMAL = 0.65
SPEED_SLOW = 0.25      # ha, nhac vat
SPEED_ROTATE = 0.45    # doan xoay J1 90 do, giam tre bam quy dao J1

# Toc do toi da THAT cua tung khop (rad/s), lay tu bang joint.c cua firmware (dps_x10):
#   J1 80 do/s, J2 8 do/s, J3 25 do/s, J4 80 do/s  (joint.c ngay 2026-10-26, ti so 12/27/14/1).
# Chi ap dung khi chay voi tham so real_limits:=true. Firmware khong cho khop chay nhanh hon,
# neu node ra lenh nhanh hon thi khop tre so voi quy dao va controller bao loi.
# Sua lai khi ban phan cung thay doi bang toc do trong joint.c.
REAL_MAX_SPEED = [math.radians(80.0), math.radians(6.5), math.radians(25.0), math.radians(80.0)]
# Gia toc toi da THAT (rad/s^2), tu acc_x10 trong joint.c: J1 120, J2 10, J3 35, J4 200 do/s^2.
# Profile s(u) = 3u^2 - 2u^3 co gia toc dinh = 6 * dq / T^2, nen T >= sqrt(6 * dq / a_max).
REAL_MAX_ACC = [math.radians(120.0), math.radians(10.0), math.radians(35.0), math.radians(20.0)]
N_POINTS = 30          # so diem trong moi quy dao


class PickPlace(Node):

    def __init__(self):
        # use_sim_time lay tu dong lenh: Gazebo can --ros-args -p use_sim_time:=true
        super().__init__('pick_place_node')
        # real_limits: true = gioi han toc do tung khop theo firmware (chay robot that)
        self.declare_parameter('real_limits', True)
        self.real_limits = self.get_parameter('real_limits').value
        # Khop 5: goc xoay vat (do) so voi huong chuan, va vi tri gap / dat (m)
        self.declare_parameter('obj_yaw_deg', 0.0)
        self.declare_parameter('place_yaw_deg', 0.0)
        self.declare_parameter('obj_xy', [0.350, 0.0])
        self.declare_parameter('place_xy', [0.0, -0.320])
        self.declare_parameter('sequence', 'front')
        self.declare_parameter('hold_s', 1.0)
        self.poses = {k: list(v) for k, v in POSES.items()}
        self.wrist = {'A': 0.0, 'B': 0.0}
        self.urdf = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/robot_description', self._on_urdf, qos)
        self.get_logger().info(
            f"use_sim_time = {self.get_parameter('use_sim_time').value}, "
            f"real_limits = {self.real_limits}")
        self.arm = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.grip = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        self.wrist_ac = ActionClient(self, FollowJointTrajectory, '/wrist_controller/follow_joint_trajectory')
        self.joint_pos = {}
        self.create_subscription(JointState, '/joint_states', self._on_joint_states, 10)

    def _on_urdf(self, msg):
        self.urdf = msg.data

    def _on_joint_states(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.joint_pos[name] = pos

    # ---------------- tien ich ----------------

    def wait_ready(self):
        self.get_logger().info('Cho controller va /joint_states ...')
        for client, name in [(self.arm, 'arm_controller'), (self.grip, 'gripper_controller'),
                             (self.wrist_ac, 'wrist_controller')]:
            if not client.wait_for_server(timeout_sec=30.0):
                raise RuntimeError(f'Khong thay action cua {name}. Launch da chay xong chua?')
        while rclpy.ok() and not all(j in self.joint_pos for j in ARM_JOINTS + [GRIPPER_JOINT, WRIST_JOINT]):
            rclpy.spin_once(self, timeout_sec=0.1)
        while rclpy.ok() and self.urdf is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('San sang.')

    def plan_poses(self):
        """Tinh PRE/GRASP o A va PRE/PLACE o B bang IK 5 khop, bu do lech TCP - truc JOINT_5."""
        chain = UrdfChain(self.urdf)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        jobs = [('A', 'PRE_A', 'GRASP_A', g('obj_xy'), Z_GRASP, g('obj_yaw_deg'))]
        if g('sequence') == 'ab':
            jobs.append(('B', 'PRE_B', 'PLACE_B', g('place_xy'), Z_PLACE, g('place_yaw_deg')))
        for side, pre, low, xy, z_low, yaw_deg in jobs:
            # Huong chuan = huong ngon o tu the bang POSES voi JOINT_5 = 0
            yaw0 = grip_yaw(chain.fk(dict(zip(IK_JOINTS, POSES[low] + [0.0]))))
            yaw = yaw0 + math.radians(yaw_deg)
            q_low = plan_tcp(chain, [xy[0], xy[1], z_low], yaw, POSES[low])
            q_pre = plan_tcp(chain, [xy[0], xy[1], Z_PRE], yaw, POSES[pre])
            for name, (q, e_pos, e_tilt, e_yaw) in [(pre, q_pre), (low, q_low)]:
                if e_pos > 1.0 or e_tilt > 0.5 or e_yaw > 0.5:
                    raise RuntimeError(
                        f'IK {name} khong toi duoc (lech {e_pos:.1f} mm, nghieng {e_tilt:.2f} do, yaw {e_yaw:.2f} do). '
                        'Thuong do J1 cham gioi han +-90 do khi o B. Doi goc xoay hoac dich diem dat.')
                self.poses[name] = q[:4]
                self.get_logger().info(
                    f'{name}: J1..J4 = {[round(v, 4) for v in q[:4]]}, J5 = {math.degrees(q[4]):.1f} do '
                    f'(lech {e_pos:.2f} mm)')
            # Ha thang dung: PRE va GRASP cung J5
            if abs(q_pre[0][4] - q_low[0][4]) > math.radians(1.0):
                raise RuntimeError(f'J5 o {pre} va {low} khac nhau, khong ha thang duoc')
            self.wrist[side] = q_low[0][4]
            if side == 'A':
                self.poses['CARRY_F'][0] = self.poses[pre][0]   # J1 khong xoay khi thu ve CARRY_F
                # [03/11] Dat lai dung cho cu voi J5 = 0 (huong chuan yaw0): vat duoc nan thang khi dat.
                # PRE_P / PLACE_A tinh rieng vi J5 = 0 lam TCP lech 18 mm so voi luc gap.
                # PLACE_A cao hon diem gap 2 mm de khong ep vat xuong ban.
                for name, z in [('PRE_P', Z_PRE), ('PLACE_A', Z_PLACE)]:
                    q, e_pos, e_tilt, e_yaw = plan_tcp(chain, [xy[0], xy[1], z], yaw0, POSES[low])
                    if e_pos > 1.0 or e_tilt > 0.5 or e_yaw > 0.5 or abs(q[4]) > math.radians(1.0):
                        raise RuntimeError(f'IK {name} voi J5 = 0 khong toi duoc (lech {e_pos:.1f} mm)')
                    self.poses[name] = q[:4]
                    self.get_logger().info(
                        f'{name}: J1..J4 = {[round(v, 4) for v in q[:4]]}, J5 = {math.degrees(q[4]):.1f} do')

    def wait_sim(self, seconds):
        """Cho theo thoi gian mo phong (Gazebo co the chay cham hon thoi gian thuc)."""
        end = self.get_clock().now() + Duration(seconds=seconds)
        while rclpy.ok() and self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _send(self, client, joints, points):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joints
        goal.trajectory.points = points
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f'Controller tu choi quy dao cho {joints}')
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            # Dung ca chuoi: khong dong kep khi tay chua toi dung vi tri
            raise RuntimeError(
                f'Controller bao loi {result.error_code} cho {joints}: {result.error_string}. '
                'Dung chuoi pick and place.')
        return result.error_code

    @staticmethod
    def _smooth_trajectory(start, target, duration):
        """Noi suy thang trong khong gian khop, profile s(u) = 3u^2 - 2u^3.
        Van toc bang 0 o dau va cuoi doan, van toc dinh = 1.5 * dq / T."""
        points = []
        for i in range(1, N_POINTS + 1):
            u = i / N_POINTS
            s = 3 * u ** 2 - 2 * u ** 3
            ds = 6 * u * (1 - u) / duration
            p = JointTrajectoryPoint()
            p.positions = [a + (b - a) * s for a, b in zip(start, target)]
            p.velocities = [(b - a) * ds for a, b in zip(start, target)]
            t = duration * u
            p.time_from_start = DurationMsg(sec=int(t), nanosec=int((t % 1) * 1e9))
            points.append(p)
        return points

    # ---------------- lenh chuyen dong ----------------

    def move_arm(self, pose_name, speed=SPEED_NORMAL):
        rclpy.spin_once(self, timeout_sec=0.05)
        start = [self.joint_pos[j] for j in ARM_JOINTS]
        target = self.poses[pose_name]
        # MOT thoi gian chung cho ca 4 khop, do khop cham nhat quyet dinh (theo ca toc do va gia toc).
        # Moi khop di het quang duong cua minh trong dung thoi gian do, nen 4 khop BAT DAU va
        # TOI NOI CUNG LUC. Khop nhanh (J1, J4) tu dong chay cham lai cho khop cham (J2).
        inf = [float('inf')] * len(ARM_JOINTS)
        vlim = REAL_MAX_SPEED if self.real_limits else inf
        alim = REAL_MAX_ACC if self.real_limits else inf
        duration = 1.0
        for a, b, vmax, amax in zip(start, target, vlim, alim):
            dq = abs(b - a)
            duration = max(duration, 1.5 * dq / min(speed, vmax), math.sqrt(6.0 * dq / amax))
        self.get_logger().info(f'Tay -> {pose_name}  ({duration:.1f} s mo phong)')
        self._send(self.arm, ARM_JOINTS, self._smooth_trajectory(start, target, duration))

    def move_wrist(self, value):
        rclpy.spin_once(self, timeout_sec=0.05)
        start = [self.joint_pos[WRIST_JOINT]]
        duration = max(0.5, 1.5 * abs(value - start[0]) / WRIST_SPEED)
        self.get_logger().info(f'Khop 5 -> {math.degrees(value):.1f} do ({duration:.1f} s)')
        self._send(self.wrist_ac, [WRIST_JOINT], self._smooth_trajectory(start, [value], duration))

    def move_gripper(self, value, duration=1.5):
        rclpy.spin_once(self, timeout_sec=0.05)
        start = [self.joint_pos[GRIPPER_JOINT]]
        label = 'MO' if value >= GRIPPER_OPEN - 1e-3 else 'KEP'
        self.get_logger().info(f'Kep -> {label} ({value:.3f} rad)')
        self._send(self.grip, [GRIPPER_JOINT], self._smooth_trajectory(start, [value], duration))

    # ---------------- chuoi pick and place ----------------

    def run(self):
        self.wait_ready()
        self.plan_poses()

        self.move_gripper(GRIPPER_OPEN)
        self.move_arm('HOME')

        if self.get_parameter('sequence').value != 'ab':
            self.run_front()
            return

        # Gap tai A. Xoay khop 5 truoc khi xuong (tren khong), PRE_A da tinh kem goc nay
        self.move_wrist(self.wrist['A'])
        self.move_arm('PRE_A')
        self.move_arm('GRASP_A', SPEED_SLOW)
        self.move_gripper(GRIPPER_GRASP)
        self.wait_sim(1.0)                      # cho luc kep on dinh
        self.move_arm('PRE_A', SPEED_SLOW)

        # Dua vat len cao, xoay sang B (chi J1 thay doi)
        self.move_arm('CARRY_A', SPEED_ROTATE)    # dong 192, truoc la self.move_arm('CARRY_A')
        self.move_arm('CARRY_B', SPEED_ROTATE)    # dong 181, truoc la self.move_arm('CARRY_B')

        # Dat tai B: xoay vat tren khong theo goc dat roi moi xuong
        self.move_wrist(self.wrist['B'])
        self.move_arm('PRE_B')
        self.move_arm('PLACE_B', SPEED_SLOW)
        self.move_gripper(GRIPPER_OPEN)
        self.wait_sim(0.5)
        self.move_arm('PRE_B', SPEED_SLOW)

        # Ve home theo cung duong carry
        self.move_arm('CARRY_B')
        self.move_arm('CARRY_A', SPEED_ROTATE)
        self.move_wrist(0.0)
        self.move_arm('HOME')
        self.get_logger().info('Hoan thanh pick and place.')


    def run_front(self):
        """Gap vat truoc mat (J5 xoay theo vat), nhac len thi J5 ve 0, thu ve CARRY_F (J2 = 0),
        dat lai dung cho cu voi J5 = 0."""
        self.move_wrist(self.wrist['A'])
        self.move_arm('PRE_A')
        self.move_arm('GRASP_A', SPEED_SLOW)
        self.move_gripper(GRIPPER_GRASP)
        self.wait_sim(1.0)                      # cho luc kep on dinh
        self.move_arm('PRE_A', SPEED_SLOW)      # nhac vat thang len 85 mm
        self.move_wrist(0.0)                    # J5 ve 0 ngay khi nhac len, giu 0 toi luc dat
        self.move_arm('CARRY_F')                # J2 = 0, tu the giu vat khi xe chay
        self.wait_sim(self.get_parameter('hold_s').value)
        self.move_arm('PRE_P')                  # tren cho cu, J5 = 0
        self.move_arm('PLACE_A', SPEED_SLOW)    # ha xuong dung cho cu, vat thang huong chuan
        self.move_gripper(GRIPPER_OPEN)
        self.wait_sim(0.5)
        self.move_arm('PRE_P', SPEED_SLOW)
        self.move_arm('HOME')
        self.get_logger().info('Hoan thanh gap va dat lai truoc mat.')


def main():
    rclpy.init()
    node = PickPlace()
    try:
        node.run()
    except (RuntimeError, KeyboardInterrupt) as e:
        node.get_logger().error(str(e))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()