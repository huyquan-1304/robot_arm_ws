#!/usr/bin/env python3
"""Doi tu the 0 cua mot khop trong URDF ma khong lam thay doi hinh hoc.

Dung khi kiem tra A cho thay: tay that o tu the home (4 den sang) tuong ung voi
goc delta (rad) cua khop trong URDF (doc tu thanh truot joint_state_publisher_gui).
Sau khi sua, goc 0 trong URDF = tu the home that.

    origin_moi = origin_cu * R(axis, delta)
    q_moi      = q_cu - delta     (gioi han va cac group_state trong SRDF tru di delta)

Vi du (khop 2 trong robot_arm_4_dof.urdf.xacro):
    python3 urdf_zero_offset.py --rpy -1.5707963267949 0 0 --axis 0 0 -1 \
        --delta 0.12 --lower -2.618 --upper 2.618
"""
import argparse

import numpy as np


def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx                     # quy uoc URDF: roll quanh X, pitch quanh Y, yaw quanh Z (truc co dinh)


def R_to_rpy(R):
    p = np.arcsin(-np.clip(R[2, 0], -1.0, 1.0))
    if abs(np.cos(p)) > 1e-9:
        r = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(R[1, 0], R[0, 0])
    else:                                   # suy bien pitch = +-90 do
        r = 0.0
        y = np.arctan2(-R[0, 1], R[1, 1])
    return r, p, y


def axis_angle(axis, a):
    k = np.asarray(axis, dtype=float)
    k = k / np.linalg.norm(k)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * (K @ K)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rpy", nargs=3, type=float, required=True, help="origin rpy hien tai cua joint")
    ap.add_argument("--axis", nargs=3, type=float, required=True, help="axis cua joint")
    ap.add_argument("--delta", type=float, required=True, help="goc URDF (rad) ung voi tu the home that")
    ap.add_argument("--lower", type=float, help="limit lower hien tai (rad)")
    ap.add_argument("--upper", type=float, help="limit upper hien tai (rad)")
    a = ap.parse_args()

    R_old = rpy_to_R(*a.rpy)
    R_new = R_old @ axis_angle(a.axis, a.delta)
    r, p, y = R_to_rpy(R_new)

    # kiem tra: q_moi = q_cu - delta cho cung tu the voi moi q
    for q in np.linspace(-2.0, 2.0, 9):
        assert np.allclose(R_new @ axis_angle(a.axis, q), R_old @ axis_angle(a.axis, q + a.delta), atol=1e-9)

    print(f'origin rpy moi : rpy="{r:.6f} {p:.6f} {y:.6f}"')
    if a.lower is not None and a.upper is not None:
        print(f'limit moi      : lower="{a.lower - a.delta:.4f}" upper="{a.upper - a.delta:.4f}"')
    print(f"SRDF           : tru {a.delta:.4f} rad vao gia tri khop nay trong MOI group_state (tru home)")


if __name__ == "__main__":
    main()
