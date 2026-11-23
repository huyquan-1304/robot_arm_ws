#!/usr/bin/env python3
"""Kiem tra va cau hinh IMU Hiwonder IM10A (giao thuc WIT), KHONG can ROS dang chay.

Vi tri: amm_base_description/scripts/wit_imu_config.py
Tat wit_imu_node truoc (2 chuong trinh khong mo chung 1 cong duoc).

LENH
  ros2 run amm_base_description wit_imu_config.py scan
        Tim baud, do tan so tung loai goi, in gia toc trung binh (kiem tra huong gan) va do troi con quay.
        De IMU / xe DUNG YEN khi chay.
  ros2 run amm_base_description wit_imu_config.py setup
        Dat 115200 baud, 100 Hz, chi gui gia toc + toc do goc + goc, che do 6 truc (khong dung tu ke).
        Xong thi RUT USB IMU, cam lai, chay "scan" de chac chan module da LUU cau hinh.

Tuy chon: --port /dev/ttyUSB0  --baud 115200  --rate 100

Thanh ghi (WIT standard protocol): ghi = FF AA dia_chi byte_thap byte_cao
  0x69 KEY  (mo khoa: 88 B5)   0x00 SAVE (0 = luu)   0x02 RSW (loai goi)   0x03 RRATE (tan so)
  0x04 BAUD                    0x24 AXIS6 (1 = 6 truc)
Neu module khong nhan lenh: dung phan mem cau hinh cua hang tren Windows, dat 115200 baud, 100 Hz.
"""

import argparse
import math
import struct
import sys
import time

try:
    import serial
except ImportError:
    sys.exit('Thieu pyserial: sudo apt install python3-serial')

BAUDS = [9600, 115200, 230400, 460800, 921600, 57600, 38400, 19200]
BAUD_CODE = {9600: 0x02, 19200: 0x03, 38400: 0x04, 57600: 0x05, 115200: 0x06, 230400: 0x07}
RATE_CODE = {10: 0x06, 20: 0x07, 50: 0x08, 100: 0x09, 200: 0x0B}
RSW_ACC_GYRO_ANGLE = (1 << 1) | (1 << 2) | (1 << 3)          # 0x51, 0x52, 0x53
NAMES = {0x50: 'thoi gian', 0x51: 'gia toc', 0x52: 'con quay', 0x53: 'goc', 0x54: 'tu truong',
         0x55: 'cong', 0x56: 'ap suat', 0x57: 'GPS', 0x59: 'quaternion'}
G = 9.80665


def parse(buf):
    out = []
    while len(buf) >= 11:
        if buf[0] != 0x55 or not 0x50 <= buf[1] <= 0x5F:
            del buf[0]
            continue
        if (sum(buf[:10]) & 0xFF) != buf[10]:
            del buf[0]
            continue
        out.append((buf[1], struct.unpack('<4h', bytes(buf[2:10]))))
        del buf[:11]
    return out


def listen(ser, sec):
    """Doc 'sec' giay, tra ve {type: [goi]}"""
    ser.reset_input_buffer()
    buf, got = bytearray(), {}
    t_end = time.monotonic() + sec
    while time.monotonic() < t_end:
        buf += ser.read(512)
        for t, v in parse(buf):
            got.setdefault(t, []).append(v)
    return got


def open_port(port, baud):
    return serial.Serial(port, baud, timeout=0.05)


def find_baud(port, first=None):
    order = ([first] if first else []) + [b for b in BAUDS if b != first]
    for b in order:
        with open_port(port, b) as s:
            if 0x52 in listen(s, 1.2):
                return b
    return None


def write_reg(ser, addr, value):
    ser.write(bytes([0xFF, 0xAA, addr, value & 0xFF, (value >> 8) & 0xFF]))
    ser.flush()
    time.sleep(0.1)


def unlock(ser):
    write_reg(ser, 0x69, 0xB588)


def save(ser):
    write_reg(ser, 0x00, 0x0000)


def report(port, baud, sec=3.0):
    with open_port(port, baud) as s:
        got = listen(s, sec)
    print(f'\nCong {port} @ {baud} baud, do {sec:.0f} s:')
    for t in sorted(got):
        print(f'  0x{t:02X} {NAMES.get(t, "?"):10s} {len(got[t]) / sec:6.1f} Hz')
    if 0x51 in got:
        a = [sum(v[i] for v in got[0x51]) / len(got[0x51]) / 32768.0 * 16.0 * G for i in range(3)]
        print(f'  Gia toc TB (m/s2): x {a[0]:+.2f}  y {a[1]:+.2f}  z {a[2]:+.2f}  |a| {math.hypot(*a):.2f}')
        if a[2] < -8.0:
            print('    → truc Z cua module huong XUONG (mat chu up xuong): URDF imu_joint rpy = "pi 0 0" la dung')
        elif a[2] > 8.0:
            print('    → truc Z cua module huong LEN (mat chu ngua len)')
        else:
            print('    → truc Z khong thang dung: module gan nghieng?')
    if 0x52 in got:
        w = [sum(v[i] for v in got[0x52]) / len(got[0x52]) / 32768.0 * 2000.0 for i in range(3)]
        print(f'  Con quay TB (°/s): x {w[0]:+.3f}  y {w[1]:+.3f}  z {w[2]:+.3f}  (dung yen: la bias)')
    hz = len(got.get(0x52, [])) / sec
    if hz < 50:
        print(f'\n  Con quay chi {hz:.0f} Hz: qua cham cho EKF. Chay: wit_imu_config.py setup')
    return got


def cmd_setup(port, baud_target, rate):
    if baud_target not in BAUD_CODE or rate not in RATE_CODE:
        sys.exit(f'baud phai la {list(BAUD_CODE)}, rate phai la {list(RATE_CODE)}')
    b0 = find_baud(port, baud_target)
    if b0 is None:
        sys.exit('Khong tim thay IMU o bat ky baud nao. Kiem tra cap, cong, va tat wit_imu_node.')
    print(f'IMU dang chay {b0} baud')

    # 1. Doi baud (neu can)
    if b0 != baud_target:
        print(f'Doi baud {b0} → {baud_target} ...')
        with open_port(port, b0) as s:
            unlock(s)
            write_reg(s, 0x04, BAUD_CODE[baud_target])
        time.sleep(0.3)
        with open_port(port, baud_target) as s:
            ok = 0x52 in listen(s, 1.2)
        if not ok:
            # Mot so ban chi doi baud sau khi LUU
            with open_port(port, b0) as s:
                unlock(s)
                save(s)
            time.sleep(0.5)
            with open_port(port, baud_target) as s:
                ok = 0x52 in listen(s, 1.2)
        if not ok:
            sys.exit('Module khong doi baud. Dung phan mem cau hinh cua hang (Windows) de dat 115200, 100 Hz.')

    # 2. Loai goi, tan so, che do 6 truc, roi LUU (o baud moi)
    print(f'Dat goi gia toc + con quay + goc, {rate} Hz, che do 6 truc, luu ...')
    with open_port(port, baud_target) as s:
        unlock(s)
        write_reg(s, 0x02, RSW_ACC_GYRO_ANGLE)
        write_reg(s, 0x03, RATE_CODE[rate])
        write_reg(s, 0x24, 0x0001)
        save(s)
    time.sleep(0.5)

    got = report(port, baud_target)
    hz = len(got.get(0x52, [])) / 3.0
    if hz >= 0.9 * rate:
        print(f'\nXONG: con quay {hz:.0f} Hz @ {baud_target} baud.')
        print('Rut USB IMU, cam lai, chay "wit_imu_config.py scan" de chac module da luu.')
    else:
        print(f'\nCon quay {hz:.0f} Hz, chua dat {rate} Hz. Ma tan so cua module co the khac: '
              'thu --rate 200, hoac dung phan mem cua hang.')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('cmd', choices=['scan', 'setup'])
    ap.add_argument('--port', default='/dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=115200, help='baud muon dat (setup)')
    ap.add_argument('--rate', type=int, default=100, help='tan so muon dat, Hz (setup)')
    a, _ = ap.parse_known_args()          # bo qua --ros-args neu chay qua ros2 run

    try:
        if a.cmd == 'scan':
            b = find_baud(a.port, a.baud)
            if b is None:
                sys.exit(f'Khong nhan duoc goi WIT tu {a.port} o cac baud {BAUDS}. '
                         'Kiem tra cap, cong (ls /dev/ttyUSB*), va tat wit_imu_node.')
            report(a.port, b)
        else:
            cmd_setup(a.port, a.baud, a.rate)
    except serial.SerialException as e:
        sys.exit(f'Loi cong {a.port}: {e}')


if __name__ == '__main__':
    main()
