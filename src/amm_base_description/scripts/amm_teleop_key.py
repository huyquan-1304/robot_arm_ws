#!/usr/bin/env python3
"""Lai xe AMM Mecanum bang ban phim.

Giu phim de chay, tha phim thi xe dung (sau key_timeout giay).

  W / S : tien / lui            A / D : di ngang trai / phai
  Q / E : xoay trai / phai      Space : dung ngay
  + / - : tang / giam toc do    Ctrl+C: thoat

Chay:
  ros2 run amm_base_description amm_teleop_key.py
  ros2 run amm_base_description amm_teleop_key.py --ros-args -p stamped:=false   # che do velocity

Tham so:
  stamped     (true)  : true = TwistStamped (mecanum_drive_controller), false = Twist
  topic       (/cmd_vel)
  linear      (0.2)   : m/s khi tien, lui, di ngang
  angular     (0.8)   : rad/s khi xoay
  rate        (20.0)  : Hz, tan so phat lenh
  key_timeout (0.7)   : s, khong co phim moi trong khoang nay thi coi nhu da tha phim.
                        Phai lon hon do tre lap phim cua he dieu hanh (thuong 0.5 s).
  use_sim     (true)  : dung dong ho mo phong /clock. Xe that: false
"""

import select
import sys
import termios
import threading
import time
import tty

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.parameter import Parameter

# phim: (he so vx, he so vy, he so wz)
KEY_BINDINGS = {
    'w': (1, 0, 0), 's': (-1, 0, 0),
    'a': (0, 1, 0), 'd': (0, -1, 0),
    'q': (0, 0, 1), 'e': (0, 0, -1),
}
SPEED_STEP = 1.1
HELP = __doc__.split('Chay:')[0]


class TeleopKey(Node):
    def __init__(self):
        super().__init__('amm_teleop_key')
        self.stamped = self.declare_parameter('stamped', True).value
        topic = self.declare_parameter('topic', '/cmd_vel').value
        self.linear = float(self.declare_parameter('linear', 0.2).value)
        self.angular = float(self.declare_parameter('angular', 0.8).value)
        rate = float(self.declare_parameter('rate', 20.0).value)
        self.key_timeout = float(self.declare_parameter('key_timeout', 0.7).value)

        # Tem thoi gian theo dong ho mo phong: mecanum_drive_controller so tem nay voi
        # dong ho cua no de kiem tra reference_timeout. Xe that: -p use_sim:=false
        if self.declare_parameter('use_sim', True).value:
            self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        msg_type = TwistStamped if self.stamped else Twist
        self.pub = self.create_publisher(msg_type, topic, 10)
        self.period = 1.0 / rate

        self.factors = (0, 0, 0)
        self.last_key_time = 0.0
        self.lock = threading.Lock()

    def set_key(self, factors):
        with self.lock:
            self.factors = factors
            self.last_key_time = time.monotonic()

    def stop(self):
        with self.lock:
            self.factors = (0, 0, 0)
            self.last_key_time = 0.0

    def publish(self):
        with self.lock:
            if time.monotonic() - self.last_key_time > self.key_timeout:
                self.factors = (0, 0, 0)
            fx, fy, fz = self.factors
        twist = Twist()
        twist.linear.x = fx * self.linear
        twist.linear.y = fy * self.linear
        twist.angular.z = fz * self.angular
        if self.stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_footprint'
            msg.twist = twist
        else:
            msg = twist
        self.pub.publish(msg)


def read_key(timeout):
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if ready else ''


def main():
    rclpy.init()
    node = TeleopKey()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    settings = termios.tcgetattr(sys.stdin)
    print(HELP)
    print(f'Toc do: {node.linear:.2f} m/s, {node.angular:.2f} rad/s')
    try:
        tty.setraw(sys.stdin.fileno())
        next_pub = time.monotonic()
        while rclpy.ok():
            key = read_key(max(0.0, next_pub - time.monotonic())).lower()
            if key == '\x03':  # Ctrl+C
                break
            if key in KEY_BINDINGS:
                node.set_key(KEY_BINDINGS[key])
            elif key == ' ':
                node.stop()
            elif key in ('+', '='):
                node.linear *= SPEED_STEP
                node.angular *= SPEED_STEP
                print(f'\rToc do: {node.linear:.2f} m/s, {node.angular:.2f} rad/s   ', end='')
            elif key in ('-', '_'):
                node.linear /= SPEED_STEP
                node.angular /= SPEED_STEP
                print(f'\rToc do: {node.linear:.2f} m/s, {node.angular:.2f} rad/s   ', end='')

            if time.monotonic() >= next_pub:
                node.publish()
                next_pub += node.period
                if next_pub < time.monotonic():
                    next_pub = time.monotonic() + node.period
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.stop()
        node.publish()  # gui lenh dung truoc khi thoat
        print('\nDa dung xe.')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
