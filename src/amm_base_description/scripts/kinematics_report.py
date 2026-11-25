#!/usr/bin/env python3
"""Xuat bo do thi KIEM CHUNG DONG HOC tu rosbag (cho bao cao).

Vi tri: amm_base_description/scripts/kinematics_report.py

GHI BAG (tren Pi, trong luc chay goto_test hoac teleop):
  ros2 bag record -o vuong_ccw /cmd_vel /base/wheel_commands /base/wheel_states /odom /odometry/filtered /imu/data_raw

XUAT DO THI (Pi hoac may ao, khong can ROS dang chay):
  ros2 run amm_base_description kinematics_report.py ~/vuong_ccw
  ros2 run amm_base_description kinematics_report.py ~/vuong_ccw --path "square 1.0 ccw"
      --path : ve them duong DICH (cung cu phap voi goto_test.py: "goto 1 0 0", "line 1.0", "square 1.0 cw"...)
      --r 0.040 --k 0.225 : ban kinh banh, lx + ly (phai trung cau hinh controller luc ghi bag)
      --out <thu muc>     : mac dinh <bag>_report

KET QUA (thu muc _report):
  1_quy_dao_xy.png       Quy dao: dich, EKF (odom + IMU), odom (chi encoder)
  2_dong_hoc_nghich.png  Lenh 4 banh cua controller  so voi  tinh tay tu /cmd_vel bang cong thuc Mecanum
  3_bam_toc_do_banh.png  Lenh  so voi  do duoc tung banh (PID tren STM32)
  4_dong_hoc_thuan.png   vx, vy, wz: lenh | tinh tu 4 banh (dong hoc thuan) | wz do bang IMU
  5_huong_xe.png         Goc huong theo thoi gian: EKF, odom
  tom_tat.txt            Bang sai so

CONG THUC (giong mecanum_drive_controller, thu tu FL, FR, RL, RR, r = ban kinh, k = lx + ly):
  Nghich: wFL = (vx - vy - k wz)/r   wFR = (vx + vy + k wz)/r   wRL = (vx + vy - k wz)/r   wRR = (vx - vy + k wz)/r
  Thuan : vx = r (wFL + wFR + wRL + wRR)/4    vy = r (-wFL + wFR + wRL - wRR)/4    wz = r (-wFL + wFR - wRL + wRR)/(4k)

IMU gan up (truc Z huong xuong): wz_xe = -gyro_z. Dung --imu_no_flip neu gan ngua.
"""

import argparse
import math
import os
import sys

import numpy as np

JOINTS = ['front_left_wheel_joint', 'front_right_wheel_joint', 'rear_left_wheel_joint', 'rear_right_wheel_joint']
WN = ['FL', 'FR', 'RL', 'RR']
TOPICS = ['/cmd_vel', '/base/wheel_commands', '/base/wheel_states', '/odom', '/odometry/filtered', '/imu/data_raw']

# Mau (bang mau tham chieu, thu tu co dinh)
C_BLUE, C_ORANGE, C_AQUA, C_YELLOW = '#257de0', '#f47230', '#00b894', '#f2a600'
C_TXT, C_TXT2, C_GRID = '#191919', '#666666', '#e6e6e6'


# ----------------------------------------------------------------------
# Doc bag: rosbag2_py (co san trong ROS 2), neu loi thi dung thu vien rosbags (pip install rosbags)
# ----------------------------------------------------------------------
def read_bag(path):
    try:
        return _read_rosbag2_py(path)
    except Exception as e:  # khong co ROS, hoac loi plugin luu tru
        print(f'(rosbag2_py khong doc duoc: {e}. Thu thu vien rosbags...)')
    try:
        return _read_rosbags(path)
    except ImportError:
        sys.exit('Can ROS 2 (rosbag2_py) hoac: pip install --break-system-packages rosbags')


def _storage_id(path):
    meta = os.path.join(path, 'metadata.yaml')
    if os.path.isfile(meta):
        for line in open(meta):
            if 'storage_identifier' in line:
                return line.split(':', 1)[1].strip()
    return 'mcap'


def _read_rosbag2_py(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=_storage_id(path)),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    out = {t: [] for t in TOPICS}
    cls = {t: get_message(types[t]) for t in TOPICS if t in types}
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic in cls:
            out[topic].append((t * 1e-9, deserialize_message(data, cls[topic])))
    return out


def _read_rosbags(path):
    from pathlib import Path
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore
    out = {t: [] for t in TOPICS}
    with AnyReader([Path(path)], default_typestore=get_typestore(Stores.ROS2_JAZZY)) as r:
        cons = [c for c in r.connections if c.topic in out]
        for con, t, raw in r.messages(connections=cons):
            out[con.topic].append((t * 1e-9, r.deserialize(raw, con.msgtype)))
    return out


# ----------------------------------------------------------------------
def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def js_array(msgs, field):
    """JointState → t, mang (N, 4) theo thu tu FL FR RL RR."""
    t, rows = [], []
    for ts, m in msgs:
        d = dict(zip(m.name, getattr(m, field)))
        if all(j in d for j in JOINTS):
            t.append(ts)
            rows.append([d[j] for j in JOINTS])
    return np.array(t), np.array(rows)


def pose_array(msgs):
    t = np.array([ts for ts, _ in msgs])
    a = np.array([(m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation),
                   m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z) for _, m in msgs])
    return t, a


def to_start_frame(t, a, t_start):
    """Vi tri + yaw (cong don) trong khung xe LUC BAT DAU chay."""
    yaw = np.unwrap(a[:, 2])
    x0, y0, th0 = (np.interp(t_start, t, a[:, 0]), np.interp(t_start, t, a[:, 1]), np.interp(t_start, t, yaw))
    dx, dy = a[:, 0] - x0, a[:, 1] - y0
    c, s = math.cos(th0), math.sin(th0)
    return c * dx + s * dy, -s * dx + c * dy, yaw - th0


def goals_from_path(spec):
    """Danh sach dich (x, y, yaw) - cung cu phap va ket qua voi goto_test.py (khong can ROS)."""
    w = spec.split()
    cmd, args = w[0], w[1:]
    D = math.radians
    if cmd == 'goto':
        return [(float(args[0]), float(args[1]), D(float(args[2])) if len(args) > 2 else 0.0)]
    if cmd == 'line':
        return [(float(args[0]), 0.0, 0.0), (0.0, 0.0, 0.0)]
    if cmd == 'turn':
        deg = float(args[0])
        n = max(1, int(math.ceil(abs(deg) / 90.0)))
        return [(0.0, 0.0, D(deg * (i + 1) / n)) for i in range(n)]
    if cmd == 'square':
        s_ = float(args[0])
        sg = 1.0 if (args[1] if len(args) > 1 else 'ccw').lower() == 'ccw' else -1.0
        goals, yaw = [], 0.0
        for cx, cy in [(s_, 0.0), (s_, sg * s_), (0.0, sg * s_), (0.0, 0.0)]:
            goals.append((cx, cy, yaw))
            yaw += sg * math.pi / 2.0
            goals.append((cx, cy, yaw))
        return goals
    if cmd == 'square_holo':
        s_ = float(args[0])
        return [(s_, 0.0, 0.0), (s_, s_, 0.0), (0.0, s_, 0.0), (0.0, 0.0, 0.0)]
    sys.exit(f'--path khong hop le: {spec}')


def style(plt):
    plt.rcParams.update({
        'font.size': 10, 'axes.edgecolor': '#cccccc', 'axes.labelcolor': C_TXT2, 'xtick.color': C_TXT2,
        'ytick.color': C_TXT2, 'axes.spines.top': False, 'axes.spines.right': False, 'axes.grid': True,
        'grid.color': C_GRID, 'grid.linewidth': 0.8, 'axes.titleweight': 'bold', 'axes.titlecolor': C_TXT,
        'axes.titlesize': 11, 'axes.titlelocation': 'left', 'legend.frameon': False, 'lines.linewidth': 1.6,
        'figure.facecolor': 'white', 'savefig.facecolor': 'white'})


def rms(x):
    x = np.asarray(x)
    return float(np.sqrt(np.mean(x ** 2))) if x.size else float('nan')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--path', default=None, help='duong dich, cu phap goto_test, vd "square 1.0 ccw"')
    ap.add_argument('--r', type=float, default=0.040)
    ap.add_argument('--k', type=float, default=0.225)
    ap.add_argument('--out', default=None)
    ap.add_argument('--imu_no_flip', action='store_true')
    a, _ = ap.parse_known_args()

    bag = os.path.abspath(os.path.expanduser(a.bag.rstrip('/')))
    out = a.out or bag + '_report'
    os.makedirs(out, exist_ok=True)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    style(plt)

    d = read_bag(bag)
    miss = [t for t in TOPICS if not d[t]]
    if miss:
        print(f'Canh bao: bag thieu topic {miss}. Cac do thi can topic do se bo qua.')
    r, k = a.r, a.k
    sgn = 1.0 if a.imu_no_flip else -1.0

    # ---- du lieu ----
    tc = np.array([ts for ts, _ in d['/cmd_vel']])
    cmd = np.array([(m.twist.linear.x, m.twist.linear.y, m.twist.angular.z) for _, m in d['/cmd_vel']]) \
        if d['/cmd_vel'] else np.zeros((0, 3))
    tw_c, w_cmd = js_array(d['/base/wheel_commands'], 'velocity') if d['/base/wheel_commands'] else (np.array([]), None)
    tw_s, w_meas = js_array(d['/base/wheel_states'], 'velocity') if d['/base/wheel_states'] else (np.array([]), None)
    to, od = pose_array(d['/odom']) if d['/odom'] else (np.array([]), None)
    te, ek = pose_array(d['/odometry/filtered']) if d['/odometry/filtered'] else (np.array([]), None)
    ti = np.array([ts for ts, _ in d['/imu/data_raw']])
    gz = np.array([sgn * m.angular_velocity.z for _, m in d['/imu/data_raw']])

    # Moc thoi gian: lenh khac 0 dau tien (neu khong co lenh: dau bag)
    moving = np.where(np.abs(cmd).sum(axis=1) > 1e-4)[0] if len(cmd) else []
    t_start = tc[moving[0]] if len(moving) else min(x[0][0] for x in d.values() if x)
    t_end = tc[moving[-1]] + 2.0 if len(moving) else max(x[-1][0] for x in d.values() if x)
    T0 = t_start - 1.0
    summary = [f'BAG: {bag}', f'r = {r} m, k = lx + ly = {k} m, IMU {"up (lat)" if sgn < 0 else "ngua"}',
               f'Doan phan tich: {t_end - t_start:.1f} s', '']

    def win(t):
        return (t >= t_start) & (t <= t_end)

    # ---- 1. Quy dao XY ----
    if od is not None or ek is not None:
        fig, ax = plt.subplots(figsize=(6.4, 6.0), dpi=150)
        if a.path:
            g = goals_from_path(a.path)
            gx = [0.0] + [p[0] for p in g]
            gy = [0.0] + [p[1] for p in g]
            ax.plot(gx, gy, color=C_TXT2, lw=1.2, ls='--', label='Đường đích')
        lines = []
        for t_, arr, col, lab in ((te, ek, C_BLUE, 'EKF (odom + IMU)'), (to, od, C_ORANGE, 'Odom (chỉ encoder)')):
            if arr is None:
                continue
            x, y, yaw = to_start_frame(t_, arr, t_start)
            sel = t_ >= T0
            ax.plot(x[sel], y[sel], color=col, label=lab)
            ax.plot(x[-1], y[-1], 'o', color=col, ms=6, mec='white', mew=1.5)
            lines.append(f'{lab:22s} cuoi: x {x[-1] * 1000:+8.1f} mm  y {y[-1] * 1000:+8.1f} mm  '
                         f'yaw {math.degrees(yaw[-1]):+8.2f} do')
        ax.plot(0, 0, 's', color=C_TXT, ms=6, label='Xuất phát')
        ax.set_aspect('equal', adjustable='datalim')
        ax.set_xlabel('x (m) - hướng xe lúc xuất phát')
        ax.set_ylabel('y (m) - sang trái')
        ax.set_title('Quỹ đạo: EKF bám đường đích, odom tự trôi')
        ax.legend(loc='best', fontsize=9)
        fig.tight_layout()
        fig.savefig(os.path.join(out, '1_quy_dao_xy.png'))
        plt.close(fig)
        summary += ['1. QUY DAO (khung xe luc xuat phat)'] + lines + ['']

    # ---- 2. Dong hoc nghich ----
    if len(cmd) and w_cmd is not None:
        vx, vy, wz = (np.interp(tw_c, tc, cmd[:, i]) for i in range(3))
        ik = np.column_stack([(vx - vy - k * wz) / r, (vx + vy + k * wz) / r,
                              (vx + vy - k * wz) / r, (vx - vy + k * wz) / r])
        sel = win(tw_c)
        fig, axs = plt.subplots(4, 1, figsize=(10, 7.5), dpi=150, sharex=True)
        res = []
        for i in range(4):
            ax = axs[i]
            ax.plot(tw_c[sel] - T0, w_cmd[sel, i], color=C_BLUE, label='Lệnh của controller')
            ax.plot(tw_c[sel] - T0, ik[sel, i], color=C_ORANGE, ls='--', lw=1.2, label='Tính tay (công thức nghịch)')
            ax.set_ylabel(f'{WN[i]} (rad/s)')
            e = rms(w_cmd[sel, i] - ik[sel, i])
            res.append(e)
            ax.set_title(f'RMS lệch {e:.4f} rad/s', loc='right', fontsize=9, color=C_TXT2, fontweight='normal')
        fig.legend(*axs[0].get_legend_handles_labels(), loc='lower center', ncol=2, fontsize=9)
        fig.suptitle('Động học nghịch: controller tính đúng công thức Mecanum', x=0.01, ha='left', fontweight='bold', color=C_TXT, fontsize=12)
        axs[-1].set_xlabel('thời gian (s)')
        fig.tight_layout(rect=(0, 0.04, 1, 0.96))
        fig.savefig(os.path.join(out, '2_dong_hoc_nghich.png'))
        plt.close(fig)
        summary += ['2. DONG HOC NGHICH (controller so voi cong thuc)',
                    '   RMS lech (rad/s): ' + '  '.join(f'{n} {e:.4f}' for n, e in zip(WN, res)),
                    '   (lech nho = dung cong thuc. Lech lon khi lenh vuot gioi han toc do / gia toc cua controller)', '']

    # ---- 3. Bam toc do banh ----
    if w_cmd is not None and w_meas is not None:
        sel = win(tw_s)
        fig, axs = plt.subplots(4, 1, figsize=(10, 7.5), dpi=150, sharex=True)
        res = []
        for i in range(4):
            ax = axs[i]
            c_on_s = np.interp(tw_s, tw_c, w_cmd[:, i])
            ax.plot(tw_c[win(tw_c)] - T0, w_cmd[win(tw_c), i], color=C_BLUE, label='Lệnh')
            ax.plot(tw_s[sel] - T0, w_meas[sel, i], color=C_AQUA, lw=1.1, label='Đo được (encoder)')
            ax.set_ylabel(f'{WN[i]} (rad/s)')
            # sai so bam o doan toc do on dinh (lenh gan nhu khong doi)
            dc = np.abs(np.gradient(c_on_s, tw_s)) if len(tw_s) > 2 else np.zeros_like(c_on_s)
            st = sel & (dc < 0.05) & (np.abs(c_on_s) > 0.3)
            e = rms(w_meas[st, i] - c_on_s[st]) if st.any() else float('nan')
            res.append(e)
            ax.set_title(f'RMS lệch lúc ổn định {e:.3f} rad/s', loc='right', fontsize=9, color=C_TXT2,
                         fontweight='normal')
        fig.legend(*axs[0].get_legend_handles_labels(), loc='lower center', ncol=2, fontsize=9)
        fig.suptitle('Bám tốc độ bánh: PID trên STM32 (độ trễ do giới hạn gia tốc 30 RPM/s)', x=0.01, ha='left', fontweight='bold', color=C_TXT, fontsize=12)
        axs[-1].set_xlabel('thời gian (s)')
        fig.tight_layout(rect=(0, 0.04, 1, 0.96))
        fig.savefig(os.path.join(out, '3_bam_toc_do_banh.png'))
        plt.close(fig)
        summary += ['3. BAM TOC DO BANH (doan toc do on dinh)',
                    '   RMS lech (rad/s): ' + '  '.join(f'{n} {e:.3f}' for n, e in zip(WN, res)), '']

    # ---- 4. Dong hoc thuan ----
    if w_meas is not None:
        W = w_meas
        fk = np.column_stack([r * W.sum(axis=1) / 4.0,
                              r * (-W[:, 0] + W[:, 1] + W[:, 2] - W[:, 3]) / 4.0,
                              r * (-W[:, 0] + W[:, 1] - W[:, 2] + W[:, 3]) / (4.0 * k)])
        sel = win(tw_s)
        fig, axs = plt.subplots(3, 1, figsize=(10, 7.0), dpi=150, sharex=True)
        labs = [('vx (m/s)', 0), ('vy (m/s)', 1), ('wz (rad/s)', 2)]
        res = []
        for ax, (lab, i) in zip(axs, labs):
            if len(cmd):
                cs = win(tc)
                ax.plot(tc[cs] - T0, cmd[cs, i], color=C_BLUE, label='Lệnh (/cmd_vel)')
            ax.plot(tw_s[sel] - T0, fk[sel, i], color=C_ORANGE, lw=1.2, label='Tính từ 4 bánh (công thức thuận)')
            if i == 2 and len(ti):
                si = win(ti)
                # Trung binh truot 0.35 s (= 1 chu ky lac con lan 2.85 Hz) de thay xu huong, giu ban tho mo
                nwin = max(1, int(round(0.35 / np.median(np.diff(ti)))))
                f = np.convolve(gz, np.ones(nwin) / nwin, mode='same')
                n2 = max(1, int(round(0.5 / np.median(np.diff(ti)))))      # loc them lan 2 cho sach
                f = np.convolve(f, np.ones(n2) / n2, mode='same')
                ax.plot(ti[si] - T0, gz[si], color=C_AQUA, lw=0.6, alpha=0.35)
                ax.plot(ti[si] - T0, f[si], color=C_AQUA, lw=1.6, label='Đo bằng IMU (đã lọc)')
                fk_on_i = np.interp(ti, tw_s, fk[:, 2])
                e_imu = rms((f - fk_on_i)[si])
                ax.set_title(f'RMS (IMU - bánh) {e_imu:.4f} rad/s', loc='right', fontsize=9, color=C_TXT2,
                             fontweight='normal')
                res.append(('wz IMU - banh', e_imu))
            if len(cmd):
                c_on_s = np.interp(tw_s, tc, cmd[:, i])
                res.append((lab.split()[0] + ' banh - lenh', rms((fk[:, i] - c_on_s)[sel])))
            ax.set_ylabel(lab)
            if i < 2:
                lo, hi = ax.get_ylim()
                ax.set_ylim(min(lo, -0.01), max(hi, 0.01))
        h, l = [], []
        for ax_ in axs:
            for hh, ll in zip(*ax_.get_legend_handles_labels()):
                if ll not in l:
                    h.append(hh)
                    l.append(ll)
        fig.legend(h, l, loc='lower center', ncol=3, fontsize=9)
        fig.suptitle('Động học thuận: vận tốc xe tính từ bánh, so với lệnh và IMU', x=0.01, ha='left', fontweight='bold', color=C_TXT, fontsize=12)
        axs[-1].set_xlabel('thời gian (s)')
        fig.tight_layout(rect=(0, 0.04, 1, 0.96))
        fig.savefig(os.path.join(out, '4_dong_hoc_thuan.png'))
        plt.close(fig)
        summary += ['4. DONG HOC THUAN'] + [f'   RMS {n}: {e:.4f}' for n, e in res] + [
            '   (banh - lenh lon o doan tang/giam toc: do gioi han gia toc firmware, khong phai loi cong thuc)', '']

    # ---- 5. Huong xe ----
    if od is not None or ek is not None:
        fig, ax = plt.subplots(figsize=(10, 4.2), dpi=150)
        for t_, arr, col, lab in ((te, ek, C_BLUE, 'EKF (chủ yếu IMU)'), (to, od, C_ORANGE, 'Odom (encoder)')):
            if arr is None:
                continue
            _, _, yaw = to_start_frame(t_, arr, t_start)
            sel = t_ >= T0
            ax.plot(t_[sel] - T0, np.degrees(yaw[sel]), color=col, label=f'{lab}: cuối {math.degrees(yaw[-1]):+.2f}°')
        ax.set_xlabel('thời gian (s)')
        ax.set_ylabel('góc hướng (độ)')
        ax.set_title('Hướng xe theo thời gian: encoder không thấy phần xe tự xoay, IMU thấy')
        ax.legend(loc='best', fontsize=9)
        fig.tight_layout()
        fig.savefig(os.path.join(out, '5_huong_xe.png'))
        plt.close(fig)

    with open(os.path.join(out, 'tom_tat.txt'), 'w') as f:
        f.write('\n'.join(summary) + '\n')
    print('\n'.join(summary))
    print(f'Da xuat do thi vao: {out}')


if __name__ == '__main__':
    main()
