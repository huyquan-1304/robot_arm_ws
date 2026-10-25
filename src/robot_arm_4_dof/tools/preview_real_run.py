#!/usr/bin/env python3
"""Xem truoc robot THAT se chay chuoi gap dat ra sao, KHONG can ROS, khong can phan cung.

Vi tri: robot_arm_4_dof/tools/preview_real_run.py
Chay:   python3 tools/preview_real_run.py                 (doc config trong package)
        python3 tools/preview_real_run.py --out ket_qua    (luu hinh va bao cao vao thu muc ket_qua)

Ghep lai dung chuoi xu ly that, chay theo buoc 1 ms:
  1. Quy dao khop nhu pick_and_place_run.py (real_limits:=true): smoothstep, 1 thoi gian chung
  2. arm_controller lay mau 100 Hz
  3. Doi rad -> 0.1 do firmware nhu stm32_bridge_node.py (joint_sign, joint_offset_deg), gui 50 Hz
     dong goi nhi phan that (CRC, khung 0xAA 0x55)
  4. Mo hinh firmware + driver ZDT trong stm32_emulator_node.py (gioi han khop, lam tron, hinh thang)
  5. Goi STATE quay ve, doi nguoc ra rad
  6. Kiem tra nhu arm_controller: dung sai bam quy dao (trajectory) va dung sai dich (goal, goal_time)
     trong config/final.yaml. Vi pham thi DUNG chuoi, giong node gap that.

Ket qua: bao cao tung doan (thoi gian, sai lech lon nhat tung khop, loi) va hinh ve lenh / thuc te.
"""

import argparse
import math
import os
import struct
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PKG, 'node'))
from stm32_emulator_node import (  # noqa: E402
    FMT_CMD_JOINT, FMT_STATE, T_CMD_ENABLE, T_CMD_JOINT, T_CMD_PING, T_STATE,
    FirmwareModel, FrameParser, build_frame,
)

ARM = ['JOINT_1', 'JOINT_2', 'JOINT_3', 'JOINT_4']

# ---- Giong pick_and_place_run.py ----
POSES = {
    'HOME':    [0.0,     0.0,     0.0,     0.0],
    'PRE_A':   [-0.0221, -0.6063, -0.8749, -0.2686],
    'GRASP_A': [-0.0221, -0.7385, -0.5827, 0.1558],
    'CARRY_A': [-0.0354, 0.0319,  -0.2629, -0.2948],
    'CARRY_B': [1.5354,  0.0319,  -0.2629, -0.2948],
    'PRE_B':   [1.5487,  -0.6063, -0.8749, -0.2686],
    'PLACE_B': [1.5487,  -0.7335, -0.5867, 0.1468],
}
GRIP_OPEN, GRIP_GRASP = 1.141, 0.62
SPEED_NORMAL, SPEED_SLOW = 0.65, 0.25
REAL_MAX_SPEED = [math.radians(v) for v in (80.0, 2.0, 8.0, 80.0)]
REAL_MAX_ACC = [math.radians(v) for v in (120.0, 2.0, 10.0, 200.0)]
SEQUENCE = [
    ('grip', GRIP_OPEN), ('arm', 'HOME', SPEED_NORMAL),
    ('arm', 'PRE_A', SPEED_NORMAL), ('arm', 'GRASP_A', SPEED_SLOW), ('grip', GRIP_GRASP),
    ('arm', 'PRE_A', SPEED_SLOW), ('arm', 'CARRY_A', SPEED_NORMAL), ('arm', 'CARRY_B', SPEED_NORMAL),
    ('arm', 'PRE_B', SPEED_NORMAL), ('arm', 'PLACE_B', SPEED_SLOW), ('grip', GRIP_OPEN),
    ('arm', 'PRE_B', SPEED_SLOW), ('arm', 'CARRY_B', SPEED_NORMAL), ('arm', 'CARRY_A', SPEED_NORMAL),
    ('arm', 'HOME', SPEED_NORMAL),
]


def duration_for(start, target, speed):
    T = 1.0
    for a, b, v, ac in zip(start, target, REAL_MAX_SPEED, REAL_MAX_ACC):
        dq = abs(b - a)
        T = max(T, 1.5 * dq / min(speed, v), math.sqrt(6.0 * dq / ac))
    return T


def smooth(a, b, t, T):
    u = min(max(t / T, 0.0), 1.0)
    return a + (b - a) * (3 * u * u - 2 * u ** 3)


def interp(x, xp, fp):
    if x <= xp[0]:
        return fp[0]
    if x >= xp[-1]:
        return fp[-1]
    for i in range(1, len(xp)):
        if x <= xp[i]:
            return fp[i - 1] + (x - xp[i - 1]) / (xp[i] - xp[i - 1]) * (fp[i] - fp[i - 1])
    return fp[-1]


class Bridge:
    """Phan doi don vi cua stm32_bridge_node.py (khong can rclpy)."""

    def __init__(self, p):
        self.sign = [float(v) for v in p['joint_sign']]
        self.offset = [float(v) for v in p['joint_offset_deg']]
        self.crank = float(p['gripper_crank_m'])
        pairs = sorted(zip(p['gripper_gap_mm'], p['gripper_us']))
        self.gap, self.us = [a for a, _ in pairs], [b for _, b in pairs]
        self.max_jump = float(p.get('max_jump_rad', 0.2))

    def ros_to_fw(self, q):
        return [int(round((s * math.degrees(x) + o) * 10.0)) for x, s, o in zip(q, self.sign, self.offset)]

    def fw_to_ros(self, x10):
        return [math.radians((v / 10.0 - o) * s) for v, s, o in zip(x10, self.sign, self.offset)]

    def grip_to_us(self, q):
        gap = 2000.0 * self.crank * math.sin(max(0.0, min(q, math.pi / 2)))
        return int(round(interp(gap, self.gap, self.us)))

    def us_to_grip(self, us):
        p = sorted(zip(self.us, self.gap))
        gap = interp(us, [a for a, _ in p], [b for _, b in p])
        return math.asin(max(0.0, min(1.0, gap / (2000.0 * self.crank))))


def run(fw_cfg, br_cfg, tol, label):
    fw = FirmwareModel(fw_cfg)
    br = Bridge(br_cfg)
    parser = FrameParser()
    t_ms = 0
    state = [0.0] * 4
    grip_state = br.us_to_grip(fw.servo_us)
    cmd = [0.0] * 4
    grip_cmd = grip_state
    seq = 0
    log = {'t': [], 'cmd': [], 'act': []}
    report = []

    def step_ms(n=1):
        nonlocal t_ms, state, grip_state, seq
        for _ in range(n):
            t_ms += 1
            if t_ms % 20 == 0:                     # node cau noi 50 Hz
                seq = (seq + 1) & 0xFFFF
                fw_cmd = br.ros_to_fw(cmd)
                if max(abs(c - s) for c, s in zip(cmd, state)) > br.max_jump:
                    fw.feed(build_frame(T_CMD_PING))
                else:
                    fw.feed(build_frame(T_CMD_JOINT, struct.pack(FMT_CMD_JOINT, seq, *fw_cmd,
                                                                br.grip_to_us(grip_cmd))))
            frame = fw.tick_1ms()
            if frame:
                for t, p in parser.feed(frame):
                    if t == T_STATE:
                        v = struct.unpack(FMT_STATE, p)
                        state = br.fw_to_ros(v[1:5])
                        grip_state = br.us_to_grip(v[5])
            if t_ms % 10 == 0:
                log['t'].append(t_ms / 1000.0)
                log['cmd'].append(list(cmd))
                log['act'].append(list(state))

    # Bat dau: PING roi ENABLE
    fw.feed(build_frame(T_CMD_PING))
    step_ms(40)
    fw.feed(build_frame(T_CMD_ENABLE, b'\x01'))
    step_ms(40)

    jtol = [tol[j]['trajectory'] for j in ARM]
    gtol = [tol[j]['goal'] for j in ARM]
    goal_time = tol['goal_time']
    current = list(state)
    for item in SEQUENCE:
        if item[0] == 'grip':
            g0, g1, T = grip_cmd, item[1], 1.5
            t0 = t_ms
            while t_ms - t0 < T * 1000 + 300:
                grip_cmd = smooth(g0, g1, (t_ms - t0) / 1000.0, T)
                step_ms(10)
            report.append(('kep', f'{"MO" if g1 > 1 else "KEP"}', T, None, ''))
            continue
        _, name, speed = item
        target = POSES[name]
        T = duration_for(current, target, speed)
        t0 = t_ms
        max_err = [0.0] * 4
        fail = ''
        while True:
            t = (t_ms - t0) / 1000.0
            cmd = [smooth(a, b, t, T) for a, b in zip(current, target)]
            step_ms(10)                              # controller 100 Hz
            err = [abs(c - s) for c, s in zip(cmd, state)]
            max_err = [max(m, e) for m, e in zip(max_err, err)]
            if t <= T:
                bad = [i for i in range(4) if err[i] > jtol[i]]
                if bad:
                    i = bad[0]
                    fail = (f'VUOT DUNG SAI BAM QUY DAO khop {i + 1}: lech {math.degrees(err[i]):.3f} do '
                            f'> {math.degrees(jtol[i]):.3f} do tai t = {t:.1f} s')
                    break
            else:
                if all(e <= g for e, g in zip(err, gtol)):
                    break
                if t > T + goal_time:
                    i = max(range(4), key=lambda k: err[k] - gtol[k])
                    fail = (f'KHONG TOI DICH khop {i + 1}: con lech {math.degrees(err[i]):.3f} do '
                            f'sau {goal_time:.0f} s cho them')
                    break
        report.append(('tay', name, T, [math.degrees(e) for e in max_err], fail))
        if fail:
            break
        current = list(target)

    return log, report, fw.limit_events, t_ms / 1000.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='preview_out')
    ap.add_argument('--relax-limits', action='store_true',
                    help='Them 1 lan chay gia dinh da noi rong gioi han J2, J4 trong joint.c')
    ap.add_argument('--joint-sign', type=float, nargs=4, metavar=('J1', 'J2', 'J3', 'J4'),
                    help='Thu chieu quay khac voi stm32_bridge.yaml, vi du: --joint-sign 1 -1 1 1')
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    cfg_dir = os.path.join(PKG, 'config')
    fw_cfg = yaml.safe_load(open(os.path.join(cfg_dir, 'stm32_firmware_model.yaml')))['stm32_emulator']['ros__parameters']
    br_cfg = yaml.safe_load(open(os.path.join(cfg_dir, 'stm32_bridge.yaml')))['stm32_bridge']['ros__parameters']
    if args.joint_sign:
        br_cfg['joint_sign'] = list(args.joint_sign)
        print(f'joint_sign thu nghiem: {br_cfg["joint_sign"]}')
    tol = yaml.safe_load(open(os.path.join(cfg_dir, 'final.yaml')))['arm_controller']['ros__parameters']['constraints']

    runs = [('Cau hinh hien tai', fw_cfg)]
    if args.relax_limits:
        relaxed = dict(fw_cfg)
        relaxed['min_deg'] = [min(a, b) for a, b in zip(fw_cfg['min_deg'], [-90.0, -60.0, -90.0, -45.0])]
        runs.append(('Gia dinh noi rong gioi han J2 >= -60, J4 >= -45 do', relaxed))

    lines = []
    for k, (label, cfg) in enumerate(runs, 1):
        log, report, limits, total = run(cfg, br_cfg, tol, label)
        lines.append(f'=== {label} ===')
        lines.append(f'{"Doan":10s} {"T (s)":>7s}   sai lech lon nhat J1..J4 (do)        ket qua')
        for kind, name, T, errs, fail in report:
            e = '' if errs is None else '  '.join(f'{x:5.2f}' for x in errs)
            lines.append(f'{name:10s} {T:7.1f}   {e:34s} {fail or "OK"}')
        if limits:
            j = sorted({l[1] for l in limits})
            lines.append(f'Firmware bo {len(limits)} lenh ngoai gioi han, khop {j}. '
                         f'Vi du: khop {limits[0][1]} lenh {limits[0][2]:.1f} do tai {limits[0][0]:.1f} s')
        lines.append(f'Tong thoi gian mo phong: {total:.0f} s\n')
        plot(log, label, os.path.join(args.out, f'lan_chay_{k}.png'))
    text = '\n'.join(lines)
    print(text)
    open(os.path.join(args.out, 'bao_cao.txt'), 'w').write(text)


def plot(log, label, path):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return
    t = log['t']
    fig, ax = plt.subplots(4, 1, figsize=(11, 9), sharex=True)
    for i in range(4):
        c = [math.degrees(x[i]) for x in log['cmd']]
        a = [math.degrees(x[i]) for x in log['act']]
        ax[i].plot(t, c, color='#2a6fdb', lw=1.6, label='lenh (Pi)')
        ax[i].plot(t, a, color='#e8762b', lw=1.2, ls='--', label='thuc (firmware)')
        ax[i].set_ylabel(f'J{i + 1} (do)')
        ax[i].grid(alpha=0.3)
    ax[0].legend(loc='upper right', frameon=False)
    ax[0].set_title(label)
    ax[-1].set_xlabel('thoi gian (s)')
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


if __name__ == '__main__':
    main()
