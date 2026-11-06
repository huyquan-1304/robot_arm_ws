#!/usr/bin/env python3
"""Thu khop 5 trong Gazebo: chon vat THANG hoac LECH 30 do, roi chay pick_and_place_run.py.

Viec file lam:
  1. Hoi chon goc xoay vat (hoac lay tu --yaw).
  2. Dat lai khoi vat "cube" trong Gazebo dung vi tri va goc da chon (gz service set_pose).
  3. Chay pick_and_place_run.py voi obj_yaw_deg = goc do. Node tu tinh IK 5 khop,
     khop 5 xoay theo vat, gap, nhac len, ve CARRY_F, dat lai dung cho cu, ve home.

Chay (sau khi pick_and_place_sim.launch.py da khoi dong xong):
  python3 ~/ros2_ws/src/robot_arm_4_dof/node/pick_test_j5.py           # hien menu chon
  python3 ~/ros2_ws/src/robot_arm_4_dof/node/pick_test_j5.py --yaw 30  # bo qua menu

Vi tri vat mac dinh: x 0.35, y 0, z 0.195 (be cao 0.175 m). Doi bang --x --y --z neu sua be.
Goc duong = vat xoay nguoc chieu kim dong ho nhin tu tren xuong.
"""

import argparse
import math
import subprocess
import sys
import time

WORLD = 'pick_place'
MODEL = 'cube'

MENU = [
    ('Vat thang voi camera (0 do)', 0.0),
    ('Vat lech 30 do', 30.0),
    ('Vat lech -30 do', -30.0),
]


def choose_yaw():
    print('\nChon vi tri vat:')
    for i, (label, _) in enumerate(MENU, 1):
        print(f'  {i}. {label}')
    print(f'  {len(MENU) + 1}. Nhap goc khac')
    while True:
        s = input('Chon: ').strip()
        if s.isdigit() and 1 <= int(s) <= len(MENU):
            return MENU[int(s) - 1][1]
        if s == str(len(MENU) + 1):
            try:
                yaw = float(input('Goc (do, trong khoang -80 .. 80): '))
                if -80.0 <= yaw <= 80.0:
                    return yaw
            except ValueError:
                pass
        print('Khong hop le, chon lai.')


def place_cube(x, y, z, yaw_deg):
    """Dat lai khoi vat trong Gazebo. Cao hon mat be 1 mm de khong lun vao be."""
    h = math.radians(yaw_deg) / 2.0
    req = (f'name: "{MODEL}", position: {{x: {x}, y: {y}, z: {z + 0.001}}}, '
           f'orientation: {{x: 0, y: 0, z: {math.sin(h):.6f}, w: {math.cos(h):.6f}}}')
    cmd = ['gz', 'service', '-s', f'/world/{WORLD}/set_pose',
           '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
           '--timeout', '3000', '--req', req]
    out = subprocess.run(cmd, capture_output=True, text=True)
    if out.returncode != 0 or 'true' not in out.stdout:
        print('Khong dat lai duoc vat trong Gazebo. Gazebo da chay chua?')
        print(out.stdout.strip(), out.stderr.strip())
        sys.exit(1)
    print(f'Da dat vat tai x {x} y {y} z {z}, xoay {yaw_deg:.1f} do')


def main():
    ap = argparse.ArgumentParser(description='Thu khop 5: gap vat thang hoac lech goc')
    ap.add_argument('--yaw', type=float, help='goc xoay vat (do). Bo trong thi hien menu')
    ap.add_argument('--x', type=float, default=0.35)
    ap.add_argument('--y', type=float, default=0.0)
    ap.add_argument('--z', type=float, default=0.195, help='tam vat = mat be + 0.02')
    ap.add_argument('--hold', type=float, default=3.0, help='giay giu vat o CARRY_F')
    ap.add_argument('--fast', action='store_true', help='bo gioi han toc do that (de bao loi J2)')
    a = ap.parse_args()

    yaw = a.yaw if a.yaw is not None else choose_yaw()
    if abs(yaw) > 80.0:
        print('Goc qua lon: khop 5 chi xoay +-90 do')
        sys.exit(1)

    place_cube(a.x, a.y, a.z, yaw)
    time.sleep(1.0)   # cho vat nam yen tren be

    cmd = ['ros2', 'run', 'robot_arm_4_dof', 'pick_and_place_run.py', '--ros-args',
           '-p', f'obj_yaw_deg:={yaw}',
           '-p', f'obj_xy:=[{a.x}, {a.y}]',
           '-p', f'hold_s:={a.hold}',
           '-p', f'real_limits:={"false" if a.fast else "true"}']
    print('Chay:', ' '.join(cmd))
    sys.exit(subprocess.call(cmd))


if __name__ == '__main__':
    main()