#!/usr/bin/env python3
"""Pick and place cho robot_arm_4_dof trong Gazebo Harmonic (ROS 2 Jazzy).

Gap khoi lap phuong 40 mm tren be A (truoc mat), xoay 90 do sang phai o tu the carry,
dat len be B, roi ve home.

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
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


ARM_JOINTS = ['JOINT_1', 'JOINT_2', 'JOINT_3', 'JOINT_4']
GRIPPER_JOINT = 'GRIPPER_JOINT'

# Goc khop (rad) cho tung diem dung, tinh bang dong hoc nguoc tu URDF, kep huong thang xuong
POSES = {
    'HOME':    [0.0,     0.0,     0.0,     0.0],
    'PRE_A':   [-0.0221, -0.6063, -0.8749, -0.2686],  # TCP (0.320, 0, 0.280)
    'GRASP_A': [-0.0221, -0.7385, -0.5827, 0.1558],   # TCP (0.320, 0, 0.195)
    'CARRY_A': [-0.0354, 0.0319,  -0.2629, -0.2948],  # TCP (0.200, 0, 0.320), day vat tren lidar
    'CARRY_B': [1.5354,  0.0319,  -0.2629, -0.2948],  # nhu CARRY_A, chi xoay J1
    'PRE_B':   [1.5487,  -0.6063, -0.8749, -0.2686],  # TCP (0, -0.320, 0.280)
    'PLACE_B': [1.5487,  -0.7335, -0.5867, 0.1468],   # TCP (0, -0.320, 0.197), cach mat be 2 mm
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
REAL_MAX_ACC = [math.radians(120.0), math.radians(10.0), math.radians(35.0), math.radians(200.0)]
N_POINTS = 30          # so diem trong moi quy dao


class PickPlace(Node):

    def __init__(self):
        # use_sim_time lay tu dong lenh: Gazebo can --ros-args -p use_sim_time:=true
        super().__init__('pick_place_node')
        # real_limits: true = gioi han toc do tung khop theo firmware (chay robot that)
        self.declare_parameter('real_limits', False)
        self.real_limits = self.get_parameter('real_limits').value
        self.get_logger().info(
            f"use_sim_time = {self.get_parameter('use_sim_time').value}, "
            f"real_limits = {self.real_limits}")
        self.arm = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.grip = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        self.joint_pos = {}
        self.create_subscription(JointState, '/joint_states', self._on_joint_states, 10)

    def _on_joint_states(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.joint_pos[name] = pos

    # ---------------- tien ich ----------------

    def wait_ready(self):
        self.get_logger().info('Cho controller va /joint_states ...')
        for client, name in [(self.arm, 'arm_controller'), (self.grip, 'gripper_controller')]:
            if not client.wait_for_server(timeout_sec=30.0):
                raise RuntimeError(f'Khong thay action cua {name}. Launch da chay xong chua?')
        while rclpy.ok() and not all(j in self.joint_pos for j in ARM_JOINTS + [GRIPPER_JOINT]):
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('San sang.')

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
        target = POSES[pose_name]
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

    def move_gripper(self, value, duration=1.5):
        rclpy.spin_once(self, timeout_sec=0.05)
        start = [self.joint_pos[GRIPPER_JOINT]]
        label = 'MO' if value >= GRIPPER_OPEN - 1e-3 else 'KEP'
        self.get_logger().info(f'Kep -> {label} ({value:.3f} rad)')
        self._send(self.grip, [GRIPPER_JOINT], self._smooth_trajectory(start, [value], duration))

    # ---------------- chuoi pick and place ----------------

    def run(self):
        self.wait_ready()

        self.move_gripper(GRIPPER_OPEN)
        self.move_arm('HOME')

        # Gap tai A
        self.move_arm('PRE_A')
        self.move_arm('GRASP_A', SPEED_SLOW)
        self.move_gripper(GRIPPER_GRASP)
        self.wait_sim(1.0)                      # cho luc kep on dinh
        self.move_arm('PRE_A', SPEED_SLOW)

        # Dua vat len cao, xoay sang B (chi J1 thay doi)
        self.move_arm('CARRY_A', SPEED_ROTATE)    # dong 192, truoc la self.move_arm('CARRY_A')
        self.move_arm('CARRY_B', SPEED_ROTATE)    # dong 181, truoc la self.move_arm('CARRY_B')

        # Dat tai B
        self.move_arm('PRE_B')
        self.move_arm('PLACE_B', SPEED_SLOW)
        self.move_gripper(GRIPPER_OPEN)
        self.wait_sim(0.5)
        self.move_arm('PRE_B', SPEED_SLOW)

        # Ve home theo cung duong carry
        self.move_arm('CARRY_B')
        self.move_arm('CARRY_A', SPEED_ROTATE)
        self.move_arm('HOME')
        self.get_logger().info('Hoan thanh pick and place.')


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
