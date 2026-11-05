#!/usr/bin/env python3
"""cal_to_tf.py : collect_cal.py の CSV から base_link -> camera_link の
static_transform_publisher コマンドを出力する。

使い方:
  python3 cal_to_tf.py cal_20261104_193043.csv
  python3 cal_to_tf.py cal.csv --rcols robot_x robot_y robot_z --ccols cam_x cam_y cam_z --okcol tag_ok
  python3 cal_to_tf.py cal.csv --sh run_tf.sh     # コマンドをシェルスクリプトにも保存

Kabsch は cam optical -> base_link (剛体、スケールなし)。
camera_link への変換は、camera_link -> camera_color_optical_frame の
tf2_echo の値(§5)を使う。
"""
import sys, csv, re, argparse
import numpy as np

# camera_link -> camera_color_optical_frame (tf2_echo で確認済み)
T_CO = np.array([0.0, 0.015, 0.0])                 # [m]
Q_CO = np.array([-0.500, 0.499, -0.500, 0.501])    # xyzw


def quat_to_R(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)]])


def R_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = 2.0 * np.sqrt(t + 1.0)
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def R_to_rpy(R):
    pitch = np.arcsin(-R[2, 0])
    roll = np.arctan2(R[2, 1], R[2, 2])
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return np.degrees([roll, pitch, yaw])


def kabsch(P, Q):
    """Q ≈ R P + t (剛体)。P, Q は N x 3。"""
    pc, qc = P.mean(0), Q.mean(0)
    H = (P - pc).T @ (Q - qc)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, 1, d]) @ U.T
    t = qc - R @ pc
    return R, t


def axis_of(h):
    s = re.sub(r'[\s_\[\(]*(mm|m)[\]\)]*$', '', h.lower())
    return s[-1] if s and s[-1] in 'xyz' else None


def find_cols(header, keys):
    found = {}
    for h in header:
        if any(k in h.lower() for k in keys) and axis_of(h):
            found.setdefault(axis_of(h), []).append(h)
    ok = all(a in found and len(found[a]) == 1 for a in 'xyz')
    return [found[a][0] for a in 'xyz'] if ok else None, found


def unit_scale(a, forced):
    if forced == 'mm':
        return 1e-3
    if forced == 'm':
        return 1.0
    return 1e-3 if np.median(np.abs(a)) > 5 else 1.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv')
    ap.add_argument('--rcols', nargs=3, help='P_robot の x y z の列名')
    ap.add_argument('--ccols', nargs=3, help='P_camera の x y z の列名')
    ap.add_argument('--okcol', help='タグ検出フラグの列名 (1=有効)')
    ap.add_argument('--unit', choices=['mm', 'm'], help='CSV の単位 (省略時は大きさから自動判定)')
    ap.add_argument('--sh', help='出力コマンドを保存するシェルスクリプト')
    a = ap.parse_args()

    with open(a.csv, newline='') as f:
        rd = csv.DictReader(f)
        header = rd.fieldnames
        rows = list(rd)
    print(f'CSV: {a.csv}  行数 {len(rows)}')
    print('列:', header)

    rcols, rfound = (a.rcols, None) if a.rcols else find_cols(header, ('robot', 'pr_'))
    ccols, cfound = (a.ccols, None) if a.ccols else find_cols(header, ('cam',))
    if not rcols or not ccols:
        print('\n列を自動判定できませんでした。--rcols / --ccols で指定してください。')
        print('  robot 候補:', rfound)
        print('  camera 候補:', cfound)
        sys.exit(1)
    okcol = a.okcol or next((h for h in header if h.lower() in ('tag_ok', 'ok', 'tag_seen')), None)
    print('P_robot 列:', rcols, ' P_camera 列:', ccols, ' 有効フラグ列:', okcol)

    def num(r, c):
        try:
            return float(r[c])
        except (ValueError, TypeError, KeyError):
            return float('nan')

    flags = [okcol] + [h for h in header if h.lower() in ('still', 'stable')]
    flags = [c for c in flags if c]
    print('有効判定に使う列:', flags)

    idx, P, Q = [], [], []
    for i, r in enumerate(rows):
        if any(not num(r, c) > 0 for c in flags):
            continue
        pr = [num(r, c) for c in rcols]
        pc = [num(r, c) for c in ccols]
        if not np.all(np.isfinite(pr + pc)):
            continue
        idx.append(i)
        Q.append(pr)   # base_link 側
        P.append(pc)   # camera optical 側
    P, Q = np.array(P), np.array(Q)
    n = len(P)
    print(f'有効点: {n} / {len(rows)} (行番号 0 始まり: {idx})')
    if n < 4:
        print('有効点が4未満のため中止します (3点でも解は出ますが、検証ができません)')
        sys.exit(1)

    P *= unit_scale(P, a.unit)
    Q *= unit_scale(Q, a.unit)

    R, t = kabsch(P, Q)
    res = Q - (P @ R.T + t)
    rms = np.sqrt((res ** 2).sum(1).mean())
    print(f'\nKabsch (cam optical -> base_link, 剛体): RMS {rms*1000:.2f} mm')
    print('点ごとの残差 [mm]:', np.round(np.linalg.norm(res, axis=1) * 1000, 2))
    loo = []
    for k in range(n):
        m = np.arange(n) != k
        Rk, tk = kabsch(P[m], Q[m])
        loo.append(np.linalg.norm(Q[k] - (Rk @ P[k] + tk)) * 1000)
    print('LOO (1点外して予測) [mm]:', np.round(loo, 2))
    if rms > 0.005:
        print('警告: RMS が 5 mm を超えています。入力を確認してください。')
    if n < 6:
        print('注意: 点数が少ないため、外挿の精度は保証されません。')

    # optical -> camera_link へ換算
    R_co = quat_to_R(Q_CO)                 # camera_link -> optical
    R_bc = R @ R_co.T
    t_bc = t - R_bc @ T_CO
    q = R_to_quat(R_bc)
    rpy = R_to_rpy(R_bc)
    print(f'base_link -> camera_link: t = {np.round(t_bc, 5)} m, rpy = {np.round(rpy, 1)} deg')

    cmd = ('ros2 run tf2_ros static_transform_publisher \\\n'
        f'{t_bc[0]:.5f} {t_bc[1]:.5f} {t_bc[2]:.5f} \\\n'
        f'{q[0]:.5f} {q[1]:.5f} {q[2]:.5f} {q[3]:.5f} \\\n'
        'base_link camera_link')
    print('\n以下のコマンドを、新しいターミナルで実行してください:\n')
    print(cmd)
    if a.sh:
        with open(a.sh, 'w') as f:
            f.write('#!/bin/bash\n' + cmd + '\n')
        print(f'\n(保存: {a.sh}  実行: bash {a.sh})')


if __name__ == '__main__':
    main()